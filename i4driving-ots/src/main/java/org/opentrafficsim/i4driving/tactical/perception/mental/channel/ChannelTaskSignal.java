package org.opentrafficsim.i4driving.tactical.perception.mental.channel;

import java.util.Iterator;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Predicate;

import org.djunits.value.vdouble.scalar.Length;
import org.djutils.exceptions.Try;
import org.opentrafficsim.base.parameters.ParameterTypeDouble;
import org.opentrafficsim.base.parameters.ParameterTypeLength;
import org.opentrafficsim.base.parameters.constraint.NumericConstraint;
import org.opentrafficsim.i4driving.Stateless;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable.UnderlyingDistance;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.NeighborsPerception;

/**
 * Task demand for signal (front: brake lights, left: right indicator, right: left indicator). This applies to the first leader
 * with signal, and is defined as {@code TD_signal * (1 - s/x0)}, where {@code TD_signal} is a constant, {@code s} is the
 * distance to the leader with signal and {@code x0} is the look-ahead distance.
 * @author wjschakel
 */
@Stateless
public class ChannelTaskSignal implements ChannelTask
{

    /** Signal task demand. */
    public static final ParameterTypeDouble TDSIGNAL =
            new ParameterTypeDouble("td_signal", "Signal task demand", 0.2, NumericConstraint.POSITIVEZERO);

    /** Distance discount range. */
    public static final ParameterTypeLength X0_D = ChannelMental.X0_D;

    /** Standard set of left, right and front signal task. */
    private static final Set<ChannelTask> SET =
            Set.of(new ChannelTaskSignal(LEFT), new ChannelTaskSignal(RIGHT), new ChannelTaskSignal(FRONT));

    /** Standard supplier that supplies instances for left, right and front signal task. */
    public static final Function<LanePerception, Set<ChannelTask>> SUPPLIER = (p) -> SET;

    /** Channel. */
    private final Object channel;

    /** Predicate to test the signal on a GTU. */
    private final Predicate<LaneBasedGtu> predicate;

    /** Relative lane that applies. */
    private final RelativeLane lane;

    /**
     * Constructor.
     * @param channel channel.
     */
    public ChannelTaskSignal(final Object channel)
    {
        this.channel = channel;
        if (channel.equals(FRONT))
        {
            this.predicate = new Predicate<>()
            {
                /** {@inheritDoc} */
                @Override
                public boolean test(final LaneBasedGtu t)
                {
                    return t.isBrakingLightsOn();
                }
            };
            this.lane = RelativeLane.CURRENT;
        }
        else if (channel.equals(LEFT))
        {
            this.predicate = new Predicate<>()
            {
                /** {@inheritDoc} */
                @Override
                public boolean test(final LaneBasedGtu t)
                {
                    return t.getTurnIndicatorStatus().isRightOrBoth();
                }
            };
            this.lane = RelativeLane.LEFT;
        }
        else if (channel.equals(RIGHT))
        {
            this.predicate = new Predicate<>()
            {
                /** {@inheritDoc} */
                @Override
                public boolean test(final LaneBasedGtu t)
                {
                    return t.getTurnIndicatorStatus().isLeftOrBoth();
                }
            };
            this.lane = RelativeLane.RIGHT;
        }
        else
        {
            throw new IllegalArgumentException("Channel " + channel + " is not supported signal channel.");
        }
    }

    /** {@inheritDoc} */
    @Override
    public String getId()
    {
        return String.format("signal (%s)", this.channel);
    }

    /** {@inheritDoc} */
    @Override
    public Object getChannel()
    {
        return this.channel;
    }

    /** {@inheritDoc} */
    @Override
    public double getDemand(final LanePerception perception)
    {
        NeighborsPerception neighbors = Try.assign(() -> perception.getPerceptionCategory(NeighborsPerception.class),
                "NeighborsPerception not present.");
        Iterator<UnderlyingDistance<LaneBasedGtu>> leaders = neighbors.getLeaders(this.lane).underlyingWithDistance();
        while (leaders.hasNext())
        {
            UnderlyingDistance<LaneBasedGtu> leader = leaders.next();
            if (this.predicate.test(leader.getObject()))
            {
                Length x0 =
                        Try.assign(() -> perception.getGtu().getParameters().getParameter(X0_D), "Parameter x0_d not present.");
                double tdSignal = Try.assign(() -> perception.getGtu().getParameters().getParameter(TDSIGNAL),
                        "Parameter td_signal not available.");
                return tdSignal * (1.0 - leader.getDistance().si / x0.si);
            }
        }
        return 0.0;
    }

}
