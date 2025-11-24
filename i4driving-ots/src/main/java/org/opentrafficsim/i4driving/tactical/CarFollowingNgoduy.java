package org.opentrafficsim.i4driving.tactical;

import java.util.Iterator;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djutils.exceptions.Throw;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeDouble;
import org.opentrafficsim.base.parameters.ParameterTypeInteger;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.base.parameters.constraint.NumericConstraint;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionIterable;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionIterableSet;
import org.opentrafficsim.road.gtu.lane.perception.headway.Headway;
import org.opentrafficsim.road.gtu.lane.tactical.following.CarFollowingModel;
import org.opentrafficsim.road.network.speed.SpeedLimitInfo;

/**
 * Car-following model that applies multi-leader spatial anticipation using a wrapped base car-following model.
 * @author wjschakel
 */
public class CarFollowingNgoduy implements CarFollowingModel
{

    /** Weight of leader 1. */
    public static final ParameterTypeDouble MU1 =
            new ParameterTypeDouble("mu1", "Weight of leader 1", 0.79, NumericConstraint.POSITIVE);

    /** Weight of leader 2. */
    public static final ParameterTypeDouble MU2 =
            new ParameterTypeDouble("mu2", "Weight of leader 2", 0.16, NumericConstraint.POSITIVE);

    /** Weight of leader 3 (and further). */
    public static final ParameterTypeDouble MU3 =
            new ParameterTypeDouble("mu3", "Weight of leader 3 (and further)", 0.05, NumericConstraint.POSITIVE);

    /** Number of leaders. */
    public static final ParameterTypeInteger NLEADERS = new ParameterTypeInteger("nLeaders",
            "Number of leaders for spatial anticipation.", 3, NumericConstraint.ATLEASTONE);

    /** Base car-following model. */
    final CarFollowingModel baseModel;

    /**
     * Constructor.
     * @param baseModel base car-following model
     */
    public CarFollowingNgoduy(final CarFollowingModel baseModel)
    {
        Throw.whenNull(baseModel, "baseModel");
        this.baseModel = baseModel;
    }

    @Override
    public String getName()
    {
        return "Ngoduy (" + this.baseModel.getName() + ")";
    }

    @Override
    public String getLongName()
    {
        return "Ngoduy (" + this.baseModel.getLongName() + ")";
    }

    @Override
    public Length desiredHeadway(final Parameters parameters, final Speed speed) throws ParameterException
    {
        return this.baseModel.desiredHeadway(parameters, speed);
    }

    @Override
    public Speed desiredSpeed(final Parameters parameters, final SpeedLimitInfo speedInfo) throws ParameterException
    {
        return this.baseModel.desiredSpeed(parameters, speedInfo);
    }

    @Override
    public Acceleration followingAcceleration(final Parameters parameters, final Speed speed,
            final SpeedLimitInfo speedLimitInfo, final PerceptionIterable<? extends Headway> leaders) throws ParameterException
    {
        if (leaders.isEmpty())
        {
            return this.baseModel.followingAcceleration(parameters, speed, speedLimitInfo, leaders);
        }

        // Create a single Headway object that combines the leaders
        double weightSum = 0.0;
        double dv = 0.0;
        double s = 0.0;
        int nLeaders = parameters.getParameter(NLEADERS);
        Iterator<? extends Headway> it = leaders.iterator();
        int n = 0;
        double vPrev = speed.si;
        double xPrev = 0.0;
        while (it.hasNext() && n < nLeaders)
        {
            Headway h = it.next();
            double weight = parameters.getParameter(n == 0 ? MU1 : (n == 1 ? MU2 : MU3));
            weightSum += weight;
            dv += (weight * (vPrev - h.getSpeed().si));
            s += (weight * (h.getDistance().si - xPrev));
            vPrev = h.getSpeed().si;
            if (h.getLength() != null)
            {
                xPrev = h.getDistance().si + h.getLength().si;
            }
            else if (it.hasNext())
            {
                throw new IllegalStateException("Following multiple vehicles, but intermediate leader has no length.");
            }
            n++;
        }
        Speed leaderSpeed = Speed.instantiateSI(speed.si - dv / weightSum);
        Length distance = Length.instantiateSI(s / weightSum);

        Headway h = new Headway()
        {
            /** */
            private static final long serialVersionUID = 20250617L;

            @Override
            public String getId()
            {
                return leaders.first().getId();
            }

            @Override
            public Length getLength()
            {
                return leaders.first().getLength();
            }

            @Override
            public Speed getSpeed()
            {
                return leaderSpeed;
            }

            @Override
            public Length getDistance()
            {
                return distance;
            }

            @Override
            public ObjectType getObjectType()
            {
                return leaders.first().getObjectType();
            }

            @Override
            public Acceleration getAcceleration()
            {
                return leaders.first().getAcceleration();
            }

            @Override
            public Length getOverlapFront()
            {
                return null;
            }

            @Override
            public Length getOverlapRear()
            {
                return null;
            }

            @Override
            public Length getOverlap()
            {
                return null;
            }

            @Override
            public boolean isAhead()
            {
                return true;
            }

            @Override
            public boolean isBehind()
            {
                return false;
            }

            @Override
            public boolean isParallel()
            {
                return false;
            }
        };
        return this.baseModel.followingAcceleration(parameters, speed, speedLimitInfo, new PerceptionIterableSet<Headway>(h));
    }

}
