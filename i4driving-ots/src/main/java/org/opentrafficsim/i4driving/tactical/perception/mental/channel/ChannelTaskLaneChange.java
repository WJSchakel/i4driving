package org.opentrafficsim.i4driving.tactical.perception.mental.channel;

import java.util.Set;
import java.util.function.Function;

import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeDouble;
import org.opentrafficsim.base.parameters.constraint.DualBound;
import org.opentrafficsim.i4driving.Stateless;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;

/**
 * Task demand for lane changing left and right. This is defined as the level of lane change desire if the lane change desire in
 * the relevant direction is equal or larger to the other direction. Otherwise it is 0.
 * @author wjschakel
 */
@Stateless
public final class ChannelTaskLaneChange implements ChannelTask
{

    /** Current left lane change desire. */
    private static final ParameterTypeDouble DLEFT = LmrsParameters.DLEFT;

    /** Current right lane change desire. */
    private static final ParameterTypeDouble DRIGHT = LmrsParameters.DRIGHT;

    /** Scaling on lane change desire for lane change task demand. */
    public static final ParameterTypeDouble TD_D = new ParameterTypeDouble("td_d",
            "Scaling on lane change desire for lane change task demand.", 1.0, DualBound.UNITINTERVAL);

    /** Standard set of left and right lane-change task. */
    private static final Set<ChannelTask> SET = Set.of(new ChannelTaskLaneChange(true), new ChannelTaskLaneChange(false));

    /** Standard supplier that supplies instances for left and right lane-change task. */
    public static final Function<LanePerception, Set<ChannelTask>> SUPPLIER = (p) -> SET;

    /** Whether this task instance regards the left side. */
    private final boolean left;

    /**
     * Constructor.
     * @param left whether this task instance regards the left side.
     */
    private ChannelTaskLaneChange(final boolean left)
    {
        this.left = left;
    }

    /** {@inheritDoc} */
    @Override
    public String getId()
    {
        return this.left ? "lane-changing (left)" : "lane-changing (right)";
    }

    /** {@inheritDoc} */
    @Override
    public Object getChannel()
    {
        return this.left ? LEFT : RIGHT;
    }

    /** {@inheritDoc} */
    @Override
    public double getDemand(final LanePerception perception)
    {
        try
        {
            double td = perception.getGtu().getParameters().contains(TD_D)
                    ? perception.getGtu().getParameters().getParameter(TD_D) : TD_D.getDefaultValue();
            double dLeft = perception.getGtu().getParameters().getParameter(DLEFT);
            double dRight = perception.getGtu().getParameters().getParameter(DRIGHT);
            return Math.min(0.999, td * (this.left ? td * (dLeft >= dRight && dLeft > 0.0 ? dLeft : 0.0)
                    : (dRight >= dLeft && dRight > 0.0 ? dRight : 0.0)));
        }
        catch (ParameterException ex)
        {
            throw new RuntimeException(ex);
        }
    }

}
