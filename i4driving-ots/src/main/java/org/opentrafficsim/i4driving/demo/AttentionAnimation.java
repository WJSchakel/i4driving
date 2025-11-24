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

    /** Radius around GTU along which the attention circles are placed. */
    private static final double CENTER_RADIUS = 4.5;

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
                    // find angle towards point, and place it at 45 degrees left or right according to the angle side
                    Point2d point;
                    if (channel instanceof Conflict conflict)
                    {
                        // on a conflict we take a point 10m upstream
                        double ddx = 10.0 * Math.cos(conflict.getOtherConflict().getLocation().dirZ);
                        double ddy = 10.0 * Math.sin(conflict.getOtherConflict().getLocation().dirZ);
                        point = conflict.getOtherConflict().getLocation().translate(-ddx, -ddy);
                    }
                    else
                    {
                        point = object.getLocation();
                    }
                    if (AngleUtil.normalizeAroundZero(gtu.getLocation().directionTo(point) - gtu.getLocation().dirZ) > 0.0)
                    {
                        angle = Math.PI / 4.0;
                    }
                    else
                    {
                        angle = -Math.PI / 4.0;
                    }
                }
                else
                {
                    continue;
                }
                graphics.setTransform(transform);
                // center point is not correct in OTS 1.7.5
                graphics.translate(.5 * (gtu.getFront().dx().si + gtu.getRear().dx().si), 0.0);
                graphics.rotate(-angle, 0.0, 0.0);

                // connecting line
                graphics.setColor(Color.GRAY);
                graphics.draw(new Line2D.Double(0.0, 0.0, CENTER_RADIUS - MAX_RADIUS - LINE_WIDTH, 0.0));

                // transparent background fill
                Color color = SCALE.getPaint(Math.min(1.0, perceptionDelay.si));
                graphics.setColor(new Color(color.getRed(), color.getGreen(), color.getBlue(), 48));
                graphics.fill(
                        new Ellipse2D.Double(CENTER_RADIUS - MAX_RADIUS, -MAX_RADIUS, 2.0 * MAX_RADIUS, 2.0 * MAX_RADIUS));

                // non-transparent attention fill
                graphics.setColor(color);
                double r = Math.sqrt(attention);
                graphics.fill(new Ellipse2D.Double(CENTER_RADIUS - r * MAX_RADIUS, -r * MAX_RADIUS, 2.0 * r * MAX_RADIUS,
                        2.0 * r * MAX_RADIUS));

                // edge of circle
                graphics.setColor(Color.GRAY);
                float lineWidth = LINE_WIDTH - 0.02f; // prevent tiny edges between fill and border
                graphics.draw(new Ellipse2D.Double(CENTER_RADIUS - MAX_RADIUS - .5 * lineWidth, -MAX_RADIUS - .5 * lineWidth,
                        2.0 * MAX_RADIUS + lineWidth, 2.0 * MAX_RADIUS + lineWidth));
            }
            graphics.setTransform(transform);
        }

    }

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
