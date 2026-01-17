package org.opentrafficsim.i4driving.tactical.perception;

import org.djunits.value.vdouble.scalar.Duration;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeDouble;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationHeadway;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller.BehavioralAdaptation;

/**
 * Increases the headway as behavioral adaptation. The equation is T = T_base * max(1, 1+Beta_T*(TS-TS_CRIT)).
 * @author wjschakel
 */
public class AdaptationHeadwayChannel implements BehavioralAdaptation
{

    /** Parameter for desired headway scaling. */
    public static final ParameterTypeDouble BETA_T = AdaptationHeadway.BETA_T;

    /** Critical task saturation. */
    public static final ParameterTypeDouble TS_CRIT = Fuller.TS_CRIT;

    /** Base value for the minimum desired headway. */
    private Duration t0Min;

    /** Base value for the maximum desired headway. */
    private Duration t0Max;

    /** {@inheritDoc} */
    @Override
    public void adapt(final Parameters parameters, final double taskSaturation) throws ParameterException
    {
        if (this.t0Min == null)
        {
            this.t0Min = parameters.getParameterOrNull(ParameterTypes.TMIN);
            this.t0Max = parameters.getParameterOrNull(ParameterTypes.TMAX);
        }
        double tsCrit = parameters.contains(TS_CRIT) ? parameters.getParameter(TS_CRIT) : 1.0;
        double factor = Math.max(1.0, 1.0 + parameters.getParameter(BETA_T) * (parameters.getParameter(Fuller.TS) - tsCrit));
        Duration tMin = this.t0Min.times(factor);
        Duration tMax = this.t0Max.times(factor);
        if (tMax.si <= parameters.getParameter(ParameterTypes.TMIN).si)
        {
            parameters.setParameter(ParameterTypes.TMIN, tMin);
            parameters.setParameter(ParameterTypes.TMAX, tMax);
        }
        else
        {
            parameters.setParameter(ParameterTypes.TMAX, tMax);
            parameters.setParameter(ParameterTypes.TMIN, tMin);
        }
    }

}
