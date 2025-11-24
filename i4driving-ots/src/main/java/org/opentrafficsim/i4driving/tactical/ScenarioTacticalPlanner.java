package org.opentrafficsim.i4driving.tactical;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.Time;
import org.djutils.draw.point.OrientedPoint2d;
import org.djutils.draw.point.Point2d;
import org.djutils.exceptions.Throw;
import org.djutils.exceptions.Try;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterType;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.geometry.OtsLine2d;
import org.opentrafficsim.core.geometry.OtsLine2d.FractionalFallback;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.TurnIndicatorIntent;
import org.opentrafficsim.core.gtu.perception.EgoPerception;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlan;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.core.gtu.plan.operational.Segments;
import org.opentrafficsim.core.network.LateralDirectionality;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.categories.InfrastructurePerception;
import org.opentrafficsim.road.gtu.lane.plan.operational.LaneChange;
import org.opentrafficsim.road.gtu.lane.plan.operational.LaneOperationalPlanBuilder;
import org.opentrafficsim.road.gtu.lane.plan.operational.SimpleOperationalPlan;
import org.opentrafficsim.road.gtu.lane.tactical.Blockable;
import org.opentrafficsim.road.gtu.lane.tactical.DesireBased;
import org.opentrafficsim.road.gtu.lane.tactical.Synchronizable;
import org.opentrafficsim.road.gtu.lane.tactical.following.AbstractCarFollowingModel;
import org.opentrafficsim.road.gtu.lane.tactical.following.CarFollowingModel;
import org.opentrafficsim.road.gtu.lane.tactical.following.DesiredSpeedModel;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AbstractIncentivesTacticalPlanner;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationConflicts;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationIncentive;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Cooperation;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Desire;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.GapAcceptance;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Incentive;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsData;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsUtil;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Synchronization;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Tailgating;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.speed.SpeedLimitInfo;
import org.opentrafficsim.road.network.speed.SpeedLimitProspect;

import nl.tudelft.simulation.dsol.formalisms.eventscheduling.SimEvent;

/**
 * Tactical planner that uses the LMRS, but overrides actions based on {@code Commands} typically invoked by a
 * {@code CommandsHandler}. This class is similar to the {@code Lmrs} tactical planner.
 * @author wjschakel
 */
public class ScenarioTacticalPlanner extends AbstractIncentivesTacticalPlanner implements DesireBased, Synchronizable, Blockable
{

    /**  */
    private static final long serialVersionUID = 20230426L;

    /** Time between execution of model during dead reckoning. */
    private static final Duration DEAD_RECKONING_MODEL_STEP = Duration.instantiateSI(0.5);

    /** Duration to extrapolate dead reckoning. */
    private static final Duration DEAD_RECKONING_HORIZON = Duration.instantiateSI(2.0);

    /** Lane change status. */
    private final LaneChange laneChange;

    /** LMRS data. */
    private final LmrsData lmrsData;

    /** Overruled acceleration. */
    private Acceleration accelerationCommand;

    /** Overruled indicator. */
    private LateralDirectionality indicatorCommand;

    /** Overruled lane change ability. */
    private boolean laneChangesEnabledCommand = true;

    /** Lane change command, this overrules laneChangesEnabledCommand. */
    private LateralDirectionality laneChangeCommand;

    /** Operational plan to sent to external. */
    private OperationalPlan lastIntendedPlan = null;

    /** Applies dead-reckoning to follow an external source of vehicle movement. */
    private boolean deadReckoning;

    /**
     * Whether dead-reckoning is part of a hybrid mode where the external simulation determines movement based on the intention
     * of a model plan.
     */
    private boolean hybrid;

    /** Speed for dead-reckoning. */
    private Speed deadReckoningSpeed;

    /** Acceleration for dead-reckoning. */
    private Acceleration deadReckoningAcceleration;

    /** Time of last model execution to set model parameters for surrounding vehicle while dead reckoning. */
    private Time lastDeadReckoningModelExecution;

    /** Desired speed model for when the model should be reset. */
    private DesiredSpeedModel desiredSpeedModel;

    /** Synchronization state. */
    private Synchronizable.State syncState = Synchronizable.State.NONE;

    /**
     * Constructor setting the car-following model.
     * @param carFollowingModel Car-following model.
     * @param gtu GTU
     * @param lanePerception perception
     * @param synchronization type of synchronization
     * @param cooperation type of cooperation
     * @param gapAcceptance gap-acceptance
     * @param tailgating tail gating
     */
    public ScenarioTacticalPlanner(final CarFollowingModel carFollowingModel, final LaneBasedGtu gtu,
            final LanePerception lanePerception, final Synchronization synchronization, final Cooperation cooperation,
            final GapAcceptance gapAcceptance, final Tailgating tailgating)
    {
        super(carFollowingModel, gtu, lanePerception);
        this.laneChange = Try.assign(() -> new LaneChange(gtu), "Parameter LCDUR is required.", GtuException.class);
        this.lmrsData = new LmrsData(synchronization, cooperation, gapAcceptance, tailgating);
    }

    /** {@inheritDoc} */
    @Override
    public final OperationalPlan generateOperationalPlan(final Time startTime, final OrientedPoint2d locationAtStartTime)
            throws OperationalPlanException, GtuException, NetworkException, ParameterException
    {
        if (!this.deadReckoning || this.lastDeadReckoningModelExecution == null
                || startTime.minus(this.lastDeadReckoningModelExecution).si >= DEAD_RECKONING_MODEL_STEP.si)
        {
            this.lastDeadReckoningModelExecution = this.lastDeadReckoningModelExecution == null ? startTime
                    : this.lastDeadReckoningModelExecution.plus(DEAD_RECKONING_MODEL_STEP); // prevent drift due to delay

            // obtain objects to get info
            SpeedLimitProspect slp = getPerception().getPerceptionCategory(InfrastructurePerception.class)
                    .getSpeedLimitProspect(RelativeLane.CURRENT);
            SpeedLimitInfo sli = slp.getSpeedLimitInfo(Length.ZERO);
            Parameters params = getGtu().getParameters();

            // LMRS
            SimpleOperationalPlan simplePlan = LmrsUtil.determinePlan(getGtu(), startTime, getCarFollowingModel(),
                    this.laneChange, this.lmrsData, getPerception(), getMandatoryIncentives(), getVoluntaryIncentives());

            // Lower acceleration from additional sources, consider adjacent lane when changing lane or synchronizing
            Speed speed = getPerception().getPerceptionCategory(EgoPerception.class).getSpeed();
            RelativeLane[] lanes;
            double dLeft = params.getParameterOrNull(LmrsParameters.DLEFT);
            double dRight = params.getParameterOrNull(LmrsParameters.DRIGHT);
            double dSync = params.getParameterOrNull(LmrsParameters.DSYNC);
            if (this.laneChange.isChangingLane())
            {
                lanes = new RelativeLane[] {RelativeLane.CURRENT, this.laneChange.getSecondLane(getGtu())};
            }
            else if (dLeft >= dSync && dLeft >= dRight)
            {
                lanes = new RelativeLane[] {RelativeLane.CURRENT, RelativeLane.LEFT};
            }
            else if (dRight >= dSync)
            {
                lanes = new RelativeLane[] {RelativeLane.CURRENT, RelativeLane.RIGHT};
            }
            else
            {
                lanes = new RelativeLane[] {RelativeLane.CURRENT};
            }
            for (RelativeLane lane : lanes)
            {
                // On the current lane, consider all incentives. On adjacent lanes only consider incentives beyond the distance
                // over which a lane change is not yet possible, i.e. the merge distance.
                // TODO: consider route in incentives (only if not on current lane?)
                Length mergeDistance = lane.isCurrent() ? Length.ZERO
                        : Synchronization.getMergeDistance(getPerception(), lane.getLateralDirectionality());
                for (AccelerationIncentive incentive : getAccelerationIncentives())
                {
                    incentive.accelerate(simplePlan, lane, mergeDistance, getGtu(), getPerception(), getCarFollowingModel(),
                            speed, params, sli);
                }
            }

            if (simplePlan.isLaneChange())
            {
                this.laneChange.setDesiredLaneChangeDuration(params.getParameter(ParameterTypes.LCDUR));
                // adjust lane based data in perception
            }

            // check overrules
            if (this.accelerationCommand != null)
            {
                simplePlan.setAcceleration(this.accelerationCommand);
            }
            if (!this.laneChangesEnabledCommand)
            {
                try
                {
                    Field field = SimpleOperationalPlan.class.getDeclaredField("indicatorIntent");
                    field.setAccessible(true);
                    field.set(simplePlan, TurnIndicatorIntent.NONE);
                }
                catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e)
                {
                    throw new RuntimeException(e);
                }
            }
            if (this.indicatorCommand != null && !this.indicatorCommand.isNone())
            {
                if (this.indicatorCommand.isLeft())
                {
                    simplePlan.setIndicatorIntentLeft(Length.ZERO);
                }
                else
                {
                    simplePlan.setIndicatorIntentRight(Length.ZERO);
                }
            }
            if (!this.laneChangesEnabledCommand && simplePlan.isLaneChange() && !this.laneChange.isChangingLane())
            {
                try
                {
                    Field field = SimpleOperationalPlan.class.getDeclaredField("laneChangeDirection");
                    field.setAccessible(true);
                    field.set(simplePlan, LateralDirectionality.NONE);
                }
                catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e)
                {
                    throw new RuntimeException(e);
                }
            }
            if (this.laneChangeCommand != null) // this overrules 'this.laneChangesEnabled == false'
            {
                try
                {
                    Field field = SimpleOperationalPlan.class.getDeclaredField("laneChangeDirection");
                    field.setAccessible(true);
                    field.set(simplePlan, this.laneChangeCommand);
                }
                catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e)
                {
                    throw new RuntimeException(e);
                }
                this.laneChangeCommand = null; // trigger, not a state
                this.laneChange.setDesiredLaneChangeDuration(getGtu().getParameters().getParameter(ParameterTypes.LCDUR));
            }

            // set turn indicator
            simplePlan.setTurnIndicator(getGtu());

            // create plan
            if (!this.deadReckoning || this.hybrid)
            {
                OperationalPlan operationalPlan =
                        LaneOperationalPlanBuilder.buildPlanFromSimplePlan(getGtu(), startTime, simplePlan, this.laneChange);
                this.lastIntendedPlan = operationalPlan;
                this.syncState = this.lmrsData.getSynchronizationState();
                return operationalPlan;
            }
        }

        // Dead reckoning, limit lane change desire
        Parameters params = getGtu().getParameters();
        double dLeft = params.getParameter(LmrsParameters.DLEFT);
        double dRight = params.getParameter(LmrsParameters.DRIGHT);
        double dCoop = params.getParameter(LmrsParameters.DCOOP);
        if (this.indicatorCommand != null && !this.indicatorCommand.isNone())
        {
            // Indicator, keep lane change desire above (or equal to) dCoop so others cooperate
            if (this.indicatorCommand.isLeft())
            {
                params.setParameter(LmrsParameters.DLEFT, Math.max(dLeft, dCoop));
            }
            else
            {
                params.setParameter(LmrsParameters.DRIGHT, Math.max(dRight, dCoop));
            }
            this.syncState = Synchronizable.State.INDICATING;
        }
        else
        {
            // No indicator, keep lane change desire below dCoop so others do not cooperate
            double dSync = params.getParameter(LmrsParameters.DSYNC);
            params.setParameter(LmrsParameters.DLEFT, Math.min(dLeft, .5 * (dSync + dCoop)));
            params.setParameter(LmrsParameters.DRIGHT, Math.min(dRight, .5 * (dSync + dCoop)));
            this.syncState = Synchronizable.State.NONE;
            // Note: State.SYNCHRONIZING has no meaning with dead reckoning
        }

        // Create operational plan from current position
        changeLaneOnDeadReckoning(locationAtStartTime);
        boolean toStandStill = this.deadReckoningAcceleration.lt0()
                && this.deadReckoningSpeed.si / -this.deadReckoningAcceleration.si < DEAD_RECKONING_HORIZON.si;
        double t = toStandStill ? this.deadReckoningSpeed.si / -this.deadReckoningAcceleration.si : DEAD_RECKONING_HORIZON.si;
        double distance = Math.max(1.0, this.deadReckoningSpeed.si * t + .5 * this.deadReckoningAcceleration.si * t * t);
        double x = locationAtStartTime.x + Math.cos(locationAtStartTime.dirZ) * distance;
        double y = locationAtStartTime.y - Math.sin(locationAtStartTime.dirZ) * distance;
        OtsLine2d path = new OtsLine2d(locationAtStartTime, new Point2d(x, y));
        // Segments.off takes care of standstill
        return new OperationalPlan(getGtu(), path, startTime,
                Segments.off(this.deadReckoningSpeed, DEAD_RECKONING_HORIZON, this.deadReckoningAcceleration));
    }

    /**
     * Change lane when needed as the new location is closer to an adjacent lane.
     * @param location location
     * @throws GtuException exception
     */
    private void changeLaneOnDeadReckoning(final OrientedPoint2d location) throws GtuException
    {
        Lane refLane = getGtu().getReferencePosition().lane();
        double minDistance = distance(refLane, location);
        LateralDirectionality lc = null;
        for (LateralDirectionality lat : new LateralDirectionality[] {LateralDirectionality.LEFT, LateralDirectionality.RIGHT})
        {
            for (Lane adjLane : refLane.accessibleAdjacentLanesPhysical(lat, getGtu().getType()))
            {
                double distance = distance(adjLane, location);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    lc = lat;
                }
            }
        }
        if (lc != null)
        {
            getGtu().changeLaneInstantaneously(lc);
            // set referenceLaneIndex to null to finalize any ongoing lane change
            try
            {
                Field referenceLaneIndexField = LaneBasedGtu.class.getDeclaredField("referenceLaneIndex");
                referenceLaneIndexField.setAccessible(true);
                referenceLaneIndexField.set(getGtu(), 0);
            }
            catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e)
            {
                // ignore
            }
        }
    }

    /**
     * Returns distance from the location to the lane.
     * @param lane lane
     * @param location location
     * @return distance from the location to the lane
     */
    private double distance(final Lane lane, final OrientedPoint2d location)
    {
        double frac = lane.getCenterLine().projectFractional(lane.getLink().getStartNode().getHeading(),
                lane.getLink().getEndNode().getHeading(), location.x, location.y, FractionalFallback.ENDPOINT);
        return lane.getCenterLine().getLocationFractionExtended(frac).distance(location);
    }

    /** {@inheritDoc} */
    @Override
    public final Desire getLatestDesire(final Class<? extends Incentive> incentiveClass)
    {
        return this.lmrsData.getLatestDesire(incentiveClass);
    }

    /** {@inheritDoc} */
    @Override
    public Synchronizable.State getSynchronizationState()
    {
        return this.syncState;
    }

    /** {@inheritDoc} */
    @Override
    public boolean isBlocking()
    {
        for (AccelerationIncentive acc : getAccelerationIncentives())
        {
            if (acc instanceof AccelerationConflicts)
            {
                return ((AccelerationConflicts) acc).isBlocking();
            }
        }
        return false;
    }

    /**
     * Sets a fixed acceleration.
     * @param acceleration acceleration;
     */
    public void setAcceleration(final Acceleration acceleration)
    {
        this.accelerationCommand = acceleration;
        interruptMove(getGtu().getLocation());
    }

    /**
     * Removes fixed acceleration.
     */
    public void resetAcceleration()
    {
        this.accelerationCommand = null;
        interruptMove(getGtu().getLocation());
    }

    /**
     * Sets the indicator.
     * @param indicator indicator
     * @param duration duration
     */
    public void setIndicator(final LateralDirectionality indicator, final Duration duration)
    {
        this.indicatorCommand = indicator;
        getGtu().getSimulator().scheduleEventRel(duration, this, "resetIndicator", new Object[0]);
        interruptMove(getGtu().getLocation());
    }

    /**
     * Resets the indicator.
     */
    @SuppressWarnings("unused") // scheduled
    private void resetIndicator()
    {
        this.indicatorCommand = null;
        interruptMove(getGtu().getLocation());
    }

    /**
     * Disable lane changes.
     */
    public void disableLaneChanges()
    {
        this.laneChangesEnabledCommand = false;
        interruptMove(getGtu().getLocation());
    }

    /**
     * Enable lane changes.
     */
    public void enableLaneChanges()
    {
        this.laneChangesEnabledCommand = true;
        interruptMove(getGtu().getLocation());
    }

    /**
     * Sets the desired speed.
     * @param speed desired speed.
     */
    public void setDesiredSpeed(final Speed speed)
    {
        clearCache();
        CarFollowingModel cfModel = getCarFollowingModel();
        if (getCarFollowingModel() instanceof CarFollowingNgoduy ngoduy)
        {
            cfModel = ngoduy.baseModel;
        }
        try
        {
            Field modelField = AbstractCarFollowingModel.class.getDeclaredField("desiredSpeedModel");
            modelField.setAccessible(true);
            this.desiredSpeedModel = (DesiredSpeedModel) modelField.get(cfModel);
        }
        catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e)
        {
            throw new RuntimeException(e);
        }
        setDesiredSpeedModel(cfModel, new DesiredSpeedModel()
        {
            @Override
            public Speed desiredSpeed(final Parameters parameters, final SpeedLimitInfo speedInfo) throws ParameterException
            {
                return speed;
            }
        });
        interruptMove(getGtu().getLocation());
    }

    /**
     * Reset desired speed.
     */
    public void resetDesiredSpeed()
    {
        clearCache();
        CarFollowingModel cfModel = getCarFollowingModel();
        if (cfModel instanceof CarFollowingNgoduy ngoduy)
        {
            cfModel = ngoduy.baseModel;
        }
        Throw.when(this.desiredSpeedModel == null, IllegalStateException.class,
                "Attempting to reset desired speed, but no desired speed was ever set.");
        setDesiredSpeedModel(cfModel, this.desiredSpeedModel);
        interruptMove(getGtu().getLocation());
    }

    /**
     * Clears the cache for desired speed and acceleration, so the set desired speed has effect even if a plan has been
     * calculated at the same time.
     */
    private void clearCache()
    {
        try
        {
            // clear time of cached desired speed, so a new value will be calculated
            Field speedCacheField = LaneBasedGtu.class.getDeclaredField("desiredSpeedTime");
            speedCacheField.setAccessible(true);
            speedCacheField.set(getGtu(), null);
            // clear time of cached acceleration, so a new value will be calculated
            Field accelerationCacheField = LaneBasedGtu.class.getDeclaredField("carFollowingAccelerationTime");
            accelerationCacheField.setAccessible(true);
            accelerationCacheField.set(getGtu(), null);
        }
        catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e)
        {
            throw new RuntimeException(e);
        }
    }

    /**
     * Sets the desired speed model in the car-following mode using reflection.
     * @param cfModel car-following model
     * @param desiredSpeedModel desired speed model.
     */
    private void setDesiredSpeedModel(final CarFollowingModel cfModel, final DesiredSpeedModel desiredSpeedModel)
    {
        try
        {
            Field field = AbstractCarFollowingModel.class.getDeclaredField("desiredSpeedModel");
            field.setAccessible(true);
            field.set(cfModel, desiredSpeedModel);
        }
        catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e)
        {
            throw new RuntimeException(e);
        }
    }

    /**
     * Set parameter.
     * @param parameter parameter type.
     * @param value string representation of the value.
     * @throws ParameterException when the parameter value does not comply to the type.
     */
    @SuppressWarnings("unchecked")
    public void setParameter(final String parameter, final String value) throws ParameterException
    {
        ParameterType<?> parameterType;
        try
        {
            int dot = parameter.lastIndexOf(".");
            Class<?> clazz = Class.forName(parameter.substring(0, dot));
            Field field = clazz.getDeclaredField(parameter.substring(dot + 1, parameter.length()));
            parameterType = (ParameterType<?>) field.get(null);
        }
        catch (NoSuchFieldException | ClassNotFoundException | IllegalArgumentException | IllegalAccessException e)
        {
            throw new RuntimeException(e);
        }
        if (parameterType.getValueClass().equals(Acceleration.class))
        {
            getGtu().getParameters().setParameter((ParameterType<Acceleration>) parameterType, Acceleration.valueOf(value));
        }
        else if (parameterType.getValueClass().equals(Duration.class))
        {
            getGtu().getParameters().setParameter((ParameterType<Duration>) parameterType, Duration.valueOf(value));
        }
        else if (parameterType.getValueClass().equals(Length.class))
        {
            getGtu().getParameters().setParameter((ParameterType<Length>) parameterType, Length.valueOf(value));
        }
        else if (parameterType.getValueClass().equals(Speed.class))
        {
            getGtu().getParameters().setParameter((ParameterType<Speed>) parameterType, Speed.valueOf(value));
        }
        else if (parameterType.getValueClass().equals(Time.class))
        {
            getGtu().getParameters().setParameter((ParameterType<Time>) parameterType, Time.valueOf(value));
        }
        else if (parameterType.getValueClass().equals(Double.class))
        {
            getGtu().getParameters().setParameter((ParameterType<Double>) parameterType, Double.valueOf(value));
        }
        else
        {
            throw new RuntimeException("Setting parameter of type " + parameterType.getValueClass() + " is not supported.");
        }
    }

    /**
     * Initiates a lane change.
     * @param direction lane change direction.
     */
    public void changeLane(final LateralDirectionality direction)
    {
        this.laneChangeCommand = direction;
        interruptMove(getGtu().getLocation());
    }

    /**
     * Returns the last intended operational plan, and internally sets it to null. Next calls will return null until the
     * behavioural model has run again.
     * @return last intended operational plan
     */
    public OperationalPlan pullLastIntendedPlan()
    {
        OperationalPlan plan = this.lastIntendedPlan;
        this.lastIntendedPlan = null;
        return plan;
    }

    /**
     * Invokes {@code interruptMove} on the GTU through reflection. This will cancel the scheduled move event, and trigger a new
     * move now.
     * @param location location
     */
    private void interruptMove(final OrientedPoint2d location)
    {
        try
        {
            // there's a bug in interruptMove(), so need to perform its contents indirectly
            Field field = Gtu.class.getDeclaredField("nextMoveEvent");
            field.setAccessible(true);
            @SuppressWarnings("unchecked")
            SimEvent<Duration> event = (SimEvent<Duration>) field.get(getGtu());
            getGtu().getSimulator().cancelEvent(event);

            Method move = Gtu.class.getDeclaredMethod("move", OrientedPoint2d.class);
            move.setAccessible(true);
            move.invoke(getGtu(), location);
        }
        catch (NoSuchMethodException | SecurityException | IllegalAccessException | IllegalArgumentException
                | InvocationTargetException | NoSuchFieldException e)
        {
            throw new RuntimeException(e);
        }
    }

    /**
     * Starts dead reckoning.
     * @param hybrid whether dead-reckoning is part of a hybrid mode where the external simulation determines movement based on
     *            the intention of a model plan
     */
    public void startDeadReckoning(final boolean hybrid)
    {
        this.lastDeadReckoningModelExecution = null;
        this.hybrid = hybrid;
        deadReckoning(getGtu().getLocation(), getGtu().getSpeed(), Acceleration.ZERO);
    }

    /**
     * Sets location, speed and acceleration for dead reckoning.
     * @param location location
     * @param speed speed
     * @param accel acceleration
     */
    public void deadReckoning(final OrientedPoint2d location, final Speed speed, final Acceleration accel)
    {
        this.deadReckoning = true;
        this.deadReckoningSpeed = speed;
        this.deadReckoningAcceleration = accel;
        interruptMove(location);
    }

    /**
     * Stops dead reckoning and recalculates a plan.
     */
    public void stopDeadReckoning()
    {
        this.deadReckoning = false;
        this.lastDeadReckoningModelExecution = null;
        this.hybrid = false;
        interruptMove(getGtu().getLocation());
    }

    /** {@inheritDoc} */
    @Override
    public final String toString()
    {
        return "ScenarioTacticalPlanner [mandatoryIncentives=" + getMandatoryIncentives() + ", voluntaryIncentives="
                + getVoluntaryIncentives() + ", accelerationIncentives = " + getAccelerationIncentives() + "]";
    }

}
