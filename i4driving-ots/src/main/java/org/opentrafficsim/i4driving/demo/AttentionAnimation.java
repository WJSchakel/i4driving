package org.opentrafficsim.i4driving.demo;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.image.ImageObserver;
import java.rmi.RemoteException;

import org.djunits.value.vdouble.scalar.Duration;
import org.djutils.base.AngleUtil;
import org.djutils.draw.point.Point2d;
import org.opentrafficsim.base.geometry.OtsBounds2d;
import org.opentrafficsim.base.geometry.OtsLocatable;
import org.opentrafficsim.base.geometry.OtsRenderable;
import org.opentrafficsim.draw.BoundsPaintScale;
import org.opentrafficsim.draw.DrawLevel;
import org.opentrafficsim.i4driving.demo.AttentionAnimation.ChannelAttention;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelFuller;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTask;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.network.lane.conflict.Conflict;

import nl.tudelft.simulation.naming.context.Contextualized;

/**
 * Draws circles around a GTU indicating the level of attention and perception delay.
 * @author wjschakel
 */
public class AttentionAnimation extends OtsRenderable<ChannelAttention>
{

    /** */
    private static final long serialVersionUID = 20250827L;

    /** Maximum radius of attention circles. */
    private static final double MAX_RADIUS = 1.5;

    /** Radius around GTU along which the regular attention circles are placed. */
    private static final double CENTER_RADIUS = 3.0; // 4.5

    /** Radius around GTU along which the attention circles of objects are placed. */
    private static final double CENTER_RADIUS_OBJECTS = 6.0;

    /** Line width around circle. */
    private static final float LINE_WIDTH = 0.15f;

    /** Color scale for perception delay. */
    private static final BoundsPaintScale SCALE =
            new BoundsPaintScale(new double[] {0.0, 0.25, 0.5, 0.75, 1.0}, BoundsPaintScale.GREEN_RED_DARK);

    /**
     * Constructor.
     * @param source source
     * @param contextProvider contexts
     */
    public AttentionAnimation(final ChannelAttention source, final Contextualized contextProvider)
    {
        super(source, contextProvider);
    }

    @Override
    public void paint(final Graphics2D graphics, final ImageObserver observer)
    {
        LaneBasedGtu gtu = getSource().getGtu();
        if (gtu.getTacticalPlanner().getPerception().getMental() instanceof ChannelFuller mental)
        {
            AffineTransform transform = graphics.getTransform();
            graphics.setStroke(new BasicStroke(LINE_WIDTH));
            graphics.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            for (Object channel : mental.getChannels())
            {
                double attention = mental.getAttention(channel);
                Duration perceptionDelay = mental.getPerceptionDelay(channel);
                double angle;
                double radius = CENTER_RADIUS;
                if (ChannelTask.LEFT.equals(channel))
                {
                    angle = Math.PI / 2.0;
                }
                else if (ChannelTask.FRONT.equals(channel))
                {
                    angle = 0.0;
                }
                else if (ChannelTask.RIGHT.equals(channel))
                {
                    angle = -Math.PI / 2.0;
                }
                else if (ChannelTask.REAR.equals(channel))
                {
                    angle = Math.PI;
                }
                else if (channel instanceof OtsLocatable object)
                {
                    Point2d point;
                    if (channel instanceof Conflict conflict)
                    {
                        // on a conflict we take a point 25m upstream, or the upstream conflicting node if closer
                        double x = conflict.getOtherConflict().getLongitudinalPosition().si - 25.0;
                        point = conflict.getOtherConflict().getLane().getCenterLine().getLocationExtendedSI(x < 0.0 ? 0.0 : x);
                    }
                    else
                    {
                        point = object.getLocation();
                    }
                    angle = AngleUtil.normalizeAroundZero(gtu.getLocation().directionTo(point) - gtu.getLocation().dirZ);
                    radius = CENTER_RADIUS_OBJECTS;
                }
                else
                {
                    continue;
                }
                drawAttentionCircle(graphics, gtu, transform, attention, perceptionDelay, angle, radius);
            }
            graphics.setTransform(transform);
        }

    }

    /**
     * Draws attention circle.
     * @param graphics graphics
     * @param gtu GTU
     * @param transform base transform
     * @param attention attention level
     * @param perceptionDelay perception delay
     * @param angle angle to draw circle at relative to GTU
     * @param radius center circle radius around GTU
     */
    private void drawAttentionCircle(final Graphics2D graphics, final LaneBasedGtu gtu, final AffineTransform transform,
            final double attention, final Duration perceptionDelay, final double angle, final double radius)
    {
        graphics.setTransform(transform);
        
        // center point is not correct in OTS 1.7.5
        graphics.translate(.5 * (gtu.getFront().dx().si + gtu.getRear().dx().si), 0.0);
        graphics.rotate(-angle, 0.0, 0.0);

        // connecting line
        graphics.setColor(Color.GRAY);
        graphics.draw(new Line2D.Double(0.0, 0.0, radius - MAX_RADIUS - LINE_WIDTH, 0.0));

        // transparent background fill
        Color color = SCALE.getPaint(Math.min(1.0, perceptionDelay.si));
        graphics.setColor(new Color(color.getRed(), color.getGreen(), color.getBlue(), 48));
        graphics.fill(new Ellipse2D.Double(radius - MAX_RADIUS, -MAX_RADIUS, 2.0 * MAX_RADIUS, 2.0 * MAX_RADIUS));

        // non-transparent attention fill
        graphics.setColor(color);
        double r = Math.sqrt(attention);
        graphics.fill(
                new Ellipse2D.Double(radius - r * MAX_RADIUS, -r * MAX_RADIUS, 2.0 * r * MAX_RADIUS, 2.0 * r * MAX_RADIUS));

        // edge of circle
        graphics.setColor(Color.GRAY);
        float lineWidth = LINE_WIDTH - 0.02f; // prevent tiny edges between fill and border
        graphics.draw(new Ellipse2D.Double(radius - MAX_RADIUS - .5 * lineWidth, -MAX_RADIUS - .5 * lineWidth,
                2.0 * MAX_RADIUS + lineWidth, 2.0 * MAX_RADIUS + lineWidth));
    }

    /**
     * Locatable for GTU in attention context.
     */
    public static class ChannelAttention implements OtsLocatable
    {
        /** GTU. */
        private final LaneBasedGtu gtu;

        /**
         * Constructor.
         * @param gtu GTU
         */
        public ChannelAttention(final LaneBasedGtu gtu)
        {
            this.gtu = gtu;
        }

        /**
         * Returns the GTU.
         * @return GTU
         */
        public LaneBasedGtu getGtu()
        {
            return this.gtu;
        }

        @Override
        public Point2d getLocation()
        {
            return this.gtu.getLocation();
        }

        @Override
        public OtsBounds2d getBounds()
        {
            return this.gtu.getBounds();
        }

        @Override
        public double getZ() throws RemoteException
        {
            return DrawLevel.LABEL.getZ();
        }
    }

}
