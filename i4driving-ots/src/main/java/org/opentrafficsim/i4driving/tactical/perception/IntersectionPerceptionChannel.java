package org.opentrafficsim.i4driving.tactical.perception;

import java.util.Iterator;

import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djutils.exceptions.Throw;
import org.djutils.exceptions.Try;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeLength;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.geometry.OtsGeometryException;
import org.opentrafficsim.core.geometry.OtsLine2d;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.RelativePosition;
import org.opentrafficsim.core.gtu.perception.AbstractPerceptionCategory;
import org.opentrafficsim.core.network.LateralDirectionality;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.i4driving.tactical.VisibilityLanePerception;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelMental;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.AbstractPerceptionReiterable;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.categories.IntersectionPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Anticipation;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Estimation;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.HeadwayGtuType;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayConflict;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayGtu;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayStopLine;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayTrafficLight;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.conflict.Conflict;
import org.opentrafficsim.road.network.lane.conflict.ConflictPriority;
import org.opentrafficsim.road.network.lane.conflict.ConflictRule;
import org.opentrafficsim.road.network.lane.conflict.ConflictType;
import org.opentrafficsim.road.network.lane.object.trafficlight.TrafficLight;

/**
 * This class is highly similar to DirectIntersectionPerception. It is currently awaiting changes in OTS such that a different
 * HeadwayTrafficLight can be used, which can inquire a historical traffic light color on a traffic light.
 * @author wjschakel
 */
public class IntersectionPerceptionChannel extends AbstractPerceptionCategory<LaneBasedGtu, LanePerception>
        implements IntersectionPerception
{

    /** */
    private static final long serialVersionUID = 20240925L;

    /** Mental module. */
    private final ChannelMental mental;

    /** Estimation. */
    private final Estimation estimation;

    /** Anticipation. */
    private final Anticipation anticipation;

    /** Look ahead parameter type. */
    protected static final ParameterTypeLength LOOKAHEAD = ParameterTypes.LOOKAHEAD;

    /**
     * Constructor.
     * @param perception perception.
     * @param estimation estimation.
     * @param anticipation anticipation.
     */
    public IntersectionPerceptionChannel(final LanePerception perception, final Estimation estimation,
            final Anticipation anticipation)
    {
        super(perception);
        Throw.when(!(getPerception().getMental() instanceof ChannelMental), IllegalArgumentException.class,
                "Mental module is not channel based.");
        this.mental = (ChannelMental) getPerception().getMental();
        this.estimation = estimation;
        this.anticipation = anticipation;
    }

    /** {@inheritDoc} */
    @Override
    public final PerceptionCollectable<HeadwayTrafficLight, TrafficLight> getTrafficLights(final RelativeLane lane)
    {
        return computeIfAbsent("trafficLights", () -> computeTrafficLights(lane), lane);
    }

    /** {@inheritDoc} */
    @Override
    public final PerceptionCollectable<HeadwayConflict, Conflict> getConflicts(final RelativeLane lane)
    {
        return computeIfAbsent("conflicts", () -> computeConflicts(lane), lane);
    }

    /** {@inheritDoc} */
    @Override
    public final boolean isAlongsideConflictLeft()
    {
        return computeIfAbsent("alongside", () -> computeConflictAlongside(LateralDirectionality.LEFT),
                LateralDirectionality.LEFT);
    }

    /** {@inheritDoc} */
    @Override
    public final boolean isAlongsideConflictRight()
    {
        return computeIfAbsent("alongside", () -> computeConflictAlongside(LateralDirectionality.RIGHT),
                LateralDirectionality.RIGHT);
    }

    /**
     * Compute traffic lights.
     * @param lane lane
     * @return PerceptionCollectable of traffic lights
     */
    private PerceptionCollectable<HeadwayTrafficLight, TrafficLight> computeTrafficLights(final RelativeLane lane)
    {
        Iterable<org.opentrafficsim.road.gtu.lane.perception.structure.LaneStructure.Entry<TrafficLight>> iterable =
                Try.assign(() -> getPerception().getLaneStructure().getDownstreamObjects(lane, TrafficLight.class,
                        RelativePosition.FRONT, true), "");
        Route route = Try.assign(() -> getPerception().getGtu().getStrategicalPlanner().getRoute(), "");
        return new AbstractPerceptionReiterable<>(Try.assign(() -> getGtu(), "GtuException"))
        {
            /** {@inheritDoc} */
            @Override
            protected Iterator<PrimaryIteratorEntry> primaryIterator()
            {
                Iterator<org.opentrafficsim.road.gtu.lane.perception.structure.LaneStructure.Entry<TrafficLight>> iterator =
                        iterable.iterator();
                return new Iterator<>()
                {
                    /** {@inheritDoc} */
                    @Override
                    public boolean hasNext()
                    {
                        return iterator.hasNext();
                    }

                    /** {@inheritDoc} */
                    @Override
                    public AbstractPerceptionReiterable<HeadwayTrafficLight, TrafficLight>.PrimaryIteratorEntry next()
                    {
                        org.opentrafficsim.road.gtu.lane.perception.structure.LaneStructure.Entry<TrafficLight> entry =
                                iterator.next();
                        return new PrimaryIteratorEntry(entry.object(), entry.distance());
                    }
                };
            }

            /** {@inheritDoc} */
            @Override
            protected HeadwayTrafficLight perceive(final LaneBasedGtu perceivingGtu, final TrafficLight trafficLight,
                    final Length distance) throws GtuException, ParameterException
            {
                // () -> IntersectionPerceptionChannel.this.mental.getPerceptionDelay(ChannelTask.FRONT)
                // TODO: once HeadwayTrafficLight is an interface, use HeadwayTrafficLightChannel instead
                return new HeadwayTrafficLight(trafficLight, distance,
                        trafficLight.canTurnOnRed(route, getPerception().getGtu().getType()));
            }
        };
    }

    /**
     * Compute conflicts.
     * @param lane lane
     * @return PerceptionCollectable of conflicts
     */
    private PerceptionCollectable<HeadwayConflict, Conflict> computeConflicts(final RelativeLane lane)
    {
        Iterable<org.opentrafficsim.road.gtu.lane.perception.structure.LaneStructure.Entry<Conflict>> iterable =
                Try.assign(() -> getPerception().getLaneStructure().getDownstreamObjects(lane, Conflict.class,
                        RelativePosition.FRONT, true), "");
        return new AbstractPerceptionReiterable<>(Try.assign(() -> getGtu(), "GtuException"))
        {
            /** {@inheritDoc} */
            @Override
            protected Iterator<PrimaryIteratorEntry> primaryIterator()
            {
                Iterator<org.opentrafficsim.road.gtu.lane.perception.structure.LaneStructure.Entry<Conflict>> iterator =
                        iterable.iterator();
                return new Iterator<>()
                {
                    /** {@inheritDoc} */
                    @Override
                    public boolean hasNext()
                    {
                        return iterator.hasNext();
                    }

                    /** {@inheritDoc} */
                    @Override
                    public AbstractPerceptionReiterable<HeadwayConflict, Conflict>.PrimaryIteratorEntry next()
                    {
                        org.opentrafficsim.road.gtu.lane.perception.structure.LaneStructure.Entry<Conflict> entry =
                                iterator.next();
                        return new PrimaryIteratorEntry(entry.object(), entry.distance());
                    }
                };
            }

            /** {@inheritDoc} */
            @Override
            protected HeadwayConflict perceive(final LaneBasedGtu perceivingGtu, final Conflict conflict, final Length distance)
                    throws GtuException, ParameterException
            {
                Conflict otherConflict = conflict.getOtherConflict();
                ConflictType conflictType = conflict.getConflictType();
                ConflictPriority conflictPriority = conflict.conflictPriority();
                Class<? extends ConflictRule> conflictRuleType = conflict.getConflictRule().getClass();
                String id = conflict.getId();
                Length length = conflict.getLength();
                Length conflictingLength = otherConflict.getLength();
                CrossSectionLink conflictingLink = otherConflict.getLane().getLink();

                // TODO get from link combination (needs to be a map property on the links)
                Length lookAhead = Try.assign(() -> getGtu().getParameters().getParameter(LOOKAHEAD), "Parameter not present.");
                Length conflictingVisibility;
                if (perceivingGtu.getTacticalPlanner().getPerception() instanceof VisibilityLanePerception vis)
                {
                    conflictingVisibility = Length.min(lookAhead, vis.getVisibility(perceivingGtu, otherConflict));
                }
                else
                {
                    conflictingVisibility = lookAhead;
                }
                Speed conflictingSpeedLimit;
                try
                {
                    conflictingSpeedLimit = otherConflict.getLane().getHighestSpeedLimit();
                }
                catch (NetworkException exception)
                {
                    throw new RuntimeException("GTU type not available on conflicting lane.", exception);
                }

                // TODO limit 'conflictingVisibility' to first upstream traffic light, so GTU's behind it are ignored

                HeadwayConflict headwayConflict;
                try
                {
                    HeadwayGtuType headwayGtuType = new HeadwayGtuTypeChannel(IntersectionPerceptionChannel.this.estimation,
                            IntersectionPerceptionChannel.this.anticipation,
                            () -> IntersectionPerceptionChannel.this.mental.getPerceptionDelay(conflict));
                    PerceptionCollectable<HeadwayGtu, LaneBasedGtu> upstreamConflictingGTUs =
                            otherConflict.getUpstreamGtus(getGtu(), headwayGtuType, conflictingVisibility);
                    PerceptionCollectable<HeadwayGtu, LaneBasedGtu> downstreamConflictingGTUs =
                            otherConflict.getDownstreamGtus(getGtu(), headwayGtuType, lookAhead);
                    // TODO stop lines (current models happen not to use this, but should be possible)
                    HeadwayStopLine stopLine = new HeadwayStopLine("stopLineId", Length.ZERO, conflict.getLane());
                    HeadwayStopLine conflictingStopLine =
                            new HeadwayStopLine("conflictingStopLineId", Length.ZERO, conflict.getLane());

                    Lane thisLane = conflict.getLane();
                    Lane otherLane = otherConflict.getLane();
                    Length pos1a = conflict.getLongitudinalPosition();
                    Length pos2a = otherConflict.getLongitudinalPosition();
                    Length pos1b = Length.min(pos1a.plus(conflict.getLength()), thisLane.getLength());
                    Length pos2b = Length.min(pos2a.plus(otherConflict.getLength()), otherLane.getLength());
                    OtsLine2d line1 = thisLane.getCenterLine();
                    OtsLine2d line2 = otherLane.getCenterLine();
                    double dStart = line1.getLocation(pos1a).distance(line2.getLocation(pos2a));
                    double dEnd = line1.getLocation(pos1b).distance(line2.getLocation(pos2b));
                    Length startWidth =
                            Length.instantiateSI(dStart + .5 * thisLane.getWidth(pos1a).si + .5 * otherLane.getWidth(pos2a).si);
                    Length endWidth =
                            Length.instantiateSI(dEnd + .5 * thisLane.getWidth(pos1b).si + .5 * otherLane.getWidth(pos2b).si);

                    headwayConflict = new HeadwayConflict(conflictType, conflictPriority, conflictRuleType, id, distance,
                            length, conflictingLength, upstreamConflictingGTUs, downstreamConflictingGTUs,
                            conflictingVisibility, conflictingSpeedLimit, conflictingLink,
                            HeadwayConflict.Width.linear(startWidth, endWidth), stopLine, conflictingStopLine, thisLane);

                    Length trafficLightDistance = conflict.getOtherConflict()
                            .getTrafficLightDistance(perceivingGtu.getParameters().getParameter(ParameterTypes.LOOKAHEAD));
                    if (trafficLightDistance != null && trafficLightDistance.le(lookAhead))
                    {
                        headwayConflict.setConflictingTrafficLight(trafficLightDistance, conflict.isPermitted());
                    }
                }
                catch (GtuException | OtsGeometryException | ParameterException exception)
                {
                    throw new RuntimeException("Could not create headway objects.", exception);
                }
                return headwayConflict;
            }
        };
    }

    /**
     * Compute whether there is a conflict alongside.
     * @param lat lateral directionality
     * @return whether there is a conflict alongside
     */
    private boolean computeConflictAlongside(final LateralDirectionality lat)
    {
        try
        {
            RelativeLane lane = new RelativeLane(lat, 1);
            if (getPerception().getLaneStructure().exists(lane))
            {
                Iterator<org.opentrafficsim.road.gtu.lane.perception.structure.LaneStructure.Entry<Conflict>> conflicts =
                        getPerception().getLaneStructure().getUpstreamObjects(lane, Conflict.class, RelativePosition.FRONT)
                                .iterator();
                if (conflicts.hasNext())
                {
                    org.opentrafficsim.road.gtu.lane.perception.structure.LaneStructure.Entry<Conflict> entry =
                            conflicts.next();
                    return entry.distance().si < entry.object().getLength().si + getGtu().getLength().si;
                }
            }
            return false;
        }
        catch (ParameterException exception)
        {
            throw new RuntimeException("Unexpected exception while computing conflict alongside.", exception);
        }
    }

    /** {@inheritDoc} */
    @Override
    public final String toString()
    {
        return "DirectIntersectionPerception " + cacheAsString();
    }

}
