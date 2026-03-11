package org.opentrafficsim.i4driving.tactical;

import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeAcceleration;
import org.opentrafficsim.base.parameters.ParameterTypeSpeed;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.base.parameters.constraint.NumericConstraint;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionIterable;
import org.opentrafficsim.road.gtu.lane.perception.headway.Headway;
import org.opentrafficsim.road.gtu.lane.tactical.following.AbstractCarFollowingModel;
import org.opentrafficsim.road.gtu.lane.tactical.following.DesiredHeadwayModel;
import org.opentrafficsim.road.gtu.lane.tactical.following.DesiredSpeedModel;

/**
 * Implementation of the M-IDM.
 * @author wjschakel
 */
public class IdmModified extends AbstractCarFollowingModel
{

    /** Maximum (desired) car-following acceleration. */
    private static final ParameterTypeAcceleration A = ParameterTypes.A;

    /** Maximum comfortable car-following deceleration. */
    private static final ParameterTypeAcceleration B = ParameterTypes.B;
    
    /** Maximum adjustment deceleration, e.g. when speed limit drops. */
    private static final ParameterTypeAcceleration B0 = ParameterTypes.B0;

    /** Critical speed. */
    private static final ParameterTypeSpeed VC = new ParameterTypeSpeed("v_c", "Critical speed in IDM",
            new Speed(37.58, SpeedUnit.KM_PER_HOUR), NumericConstraint.POSITIVE);

    /**
     * Constructor.
     * @param desiredHeadwayModel desired headway model
     * @param desiredSpeedModel desired speed model
     */
    public IdmModified(final DesiredHeadwayModel desiredHeadwayModel, final DesiredSpeedModel desiredSpeedModel)
    {
        super(desiredHeadwayModel, desiredSpeedModel);
    }

    @Override
    public String getName()
    {
        return "M-IDM";
    }

    @Override
    public String getLongName()
    {
        return "Modified Intelligent Driver Model";
    }

    @Override
    protected Acceleration followingAcceleration(final Parameters parameters, final Speed speed, final Speed desiredSpeed,
            final Length desiredHeadway, final PerceptionIterable<? extends Headway> leaders) throws ParameterException
    {
        double v = speed.si;
        double vRat = v / desiredSpeed.si;
        double a = parameters.getParameter(A).si;
        double b = parameters.getParameter(B).si;
        Acceleration b0 = parameters.getParameter(B0);

        if (leaders.isEmpty())
        {
            return Acceleration.max(Acceleration.instantiateSI(a * (1.0 - (vRat * vRat * vRat * vRat))), b0.neg());
        }

        Headway leader = leaders.first();
        double ss = desiredHeadway.si + v * (v - leader.getSpeed().si) / (2.0 * Math.sqrt(a * b));
        double s = leader.getDistance().si;
        double sRat = ss / s;

        if (ss <= s)
        {
            return Acceleration.instantiateSI(a * (1.0 - (vRat * vRat * vRat * vRat) - (sRat * sRat)));
        }
        else if (v <= parameters.getParameter(VC).si)
        {
            return Acceleration.instantiateSI(a * (1.0 - (sRat * sRat)));
        }

        return Acceleration.instantiateSI(Math.min(a * (1.0 - (sRat * sRat)), -b));
    }

}
