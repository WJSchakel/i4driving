package org.opentrafficsim.i4driving.tactical;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Function;

import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Duration;
import org.djutils.exceptions.Throw;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterSet;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuErrorHandler;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.perception.DirectEgoPerception;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlan;
import org.opentrafficsim.core.units.distributions.ContinuousDistSpeed;
import org.opentrafficsim.i4driving.demo.AccelerationConflictsTmp;
import org.opentrafficsim.i4driving.tactical.VisibilityLanePerception.Visibility;
import org.opentrafficsim.i4driving.tactical.perception.ActiveModePerception;
import org.opentrafficsim.i4driving.tactical.perception.AdaptationHeadwayChannel;
import org.opentrafficsim.i4driving.tactical.perception.AdaptationSpeedChannel;
import org.opentrafficsim.i4driving.tactical.perception.AdaptationUpdateTime;
import org.opentrafficsim.i4driving.tactical.perception.ConflictAnticipation;
import org.opentrafficsim.i4driving.tactical.perception.IntersectionPerceptionChannel;
import org.opentrafficsim.i4driving.tactical.perception.LocalDistractionPerception;
import org.opentrafficsim.i4driving.tactical.perception.NeighborsPerceptionChannel;
import org.opentrafficsim.i4driving.tactical.perception.SaturationEstimation;
import org.opentrafficsim.i4driving.tactical.perception.mental.CarFollowingTask;
import org.opentrafficsim.i4driving.tactical.perception.mental.LaneChangeTask;
import org.opentrafficsim.i4driving.tactical.perception.mental.LocalDistractionTask;
import org.opentrafficsim.i4driving.tactical.perception.mental.TaskManagerAr;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelFuller;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelMental;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTask;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskAcceleration;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskActiveModeCrossing;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskCarFollowing;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskConflict;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskCooperation;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskIntersection;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskLaneChange;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskLocalDistraction;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskScan;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskSignal;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskTrafficLight;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ConflictUtilTmp;
import org.opentrafficsim.road.gtu.lane.CollisionException;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.AnticipationTrafficPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.DirectInfrastructurePerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.DirectIntersectionPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Anticipation;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.DirectNeighborsPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Estimation;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.HeadwayGtuType;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.HeadwayGtuType.PerceivedHeadwayGtuType;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationHeadway;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSituationalAwareness;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSpeed;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller.BehavioralAdaptation;
import org.opentrafficsim.road.gtu.lane.perception.mental.Mental;
import org.opentrafficsim.road.gtu.lane.perception.mental.Task;
import org.opentrafficsim.road.gtu.lane.perception.mental.TaskManager;
import org.opentrafficsim.road.gtu.lane.perception.mental.TaskManager.SummativeTaskManager;
import org.opentrafficsim.road.gtu.lane.tactical.LaneBasedTacticalPlannerFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.AbstractIdm;
import org.opentrafficsim.road.gtu.lane.tactical.following.CarFollowingModel;
import org.opentrafficsim.road.gtu.lane.tactical.following.DesiredSpeedModel;
import org.opentrafficsim.road.gtu.lane.tactical.following.Idm;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlus;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveKeep;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveRoute;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSocioSpeed;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSpeedWithCourtesy;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.SocioDesiredSpeed;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Cooperation;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.GapAcceptance;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsUtil;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Synchronization;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Tailgating;

import nl.tudelft.simulation.jstats.distributions.DistLogNormal;
import nl.tudelft.simulation.jstats.distributions.DistNormalTrunc;
import nl.tudelft.simulation.jstats.distributions.DistTriangular;
import nl.tudelft.simulation.jstats.streams.StreamInterface;
import picocli.CommandLine.Option;

/**
 * Tactical planner factory for {@code ScenarioTacticalPlanner}. This supports perception with or without Fuller, based on
 * summative tasks, Anticipation Reliance or the AttentionMatrix using perception channels. Various tasks and behavioral
 * adaptations can be disabled. This factory also supports social interactions. Car-following can be either the IDM, IDM+ or
 * M-IDM. Additionally, spatial anticipation (multi-leader) can be enabled based on Ngoduy. When enabled, perception errors are
 * based on task saturation, and anticipation is constant-speed.
 * <p>
 * This class can be used in a simulation script using command line arguments by mixing it in.
 * 
 * <pre>{@code
 * @Mixin
 * private ScenarioTacticalPlannerFactory mixinModel = new ScenarioTacticalPlannerFactory();
 * }</pre>
 * </p>
 * <p>
 * Make sure that the factory receives a stream by using {@code setStream()} before any vehicle is generated.
 * </p>
 * <p>
 * The factory can be used in a single-shot mode, where all changes to the settings only apply to a single tactical planner
 * generated. This is achieved through the method {@code setSingleShotMode()}. If the factory is not in this mode, all changes
 * are permanent. The single-shot mode needs to be reset using {@code resetMode()}, after the GTU generated in single-shot mode
 * has been generated.
 * @author wjschakel
 */
public class ScenarioTacticalPlannerFactory implements LaneBasedTacticalPlannerFactory<ScenarioTacticalPlanner>
{

    /** Random number stream. */
    private StreamInterface stream;

    /** Distribution of vGain. */
    private ContinuousDistSpeed vGainDist;

    /** Distribution of sigma. */
    private DistTriangular sigmaDist;

    /** Distribution of fSpeed. */
    private DistNormalTrunc fSpeedDist;

    /** Map storing the state. */
    private Map<Field, Object> state;

    // Fuller

    /** Fuller implementation. */
    @Option(names = {"--fullerImplementation"},
            description = "Implementation of Fuller: NONE, SUMMATIVE, ANTICIPATION_RELIANCE or ATTENTION_MATRIX",
            defaultValue = "ATTENTION_MATRIX")
    private FullerImplementation fullerImplementation = FullerImplementation.ATTENTION_MATRIX;

    /** Primary task for FullerImplementation.ANTICIPATION_RELIANCE. */
    @Option(names = {"--primaryTask"}, description = "Primary task under ANTICIPATION_RELIANCE: LANE_CHANGING or CAR_FOLLOWING",
            defaultValue = "LANE_CHANGING")
    private PrimaryTask primaryTask = PrimaryTask.LANE_CHANGING;

    /** Temporal constant-speed anticipation. */
    @Option(names = {"--anticipation"}, description = "Enables temporal constant-speed anticipation.", defaultValue = "true",
            negatable = true)
    private boolean temporalAnticipation = true;

    // Tasks

    /** Car-following task. */
    @Option(names = {"--carFollowingTask"}, description = "Enables car-following task.", defaultValue = "true",
            negatable = true)
    private boolean carFollowingTask = true;

    /** Free acceleration task. */
    @Option(names = {"--freeAccelerationTask"},
            description = "Enables free acceleration task, useful when updateTimeAdaptation is true.", defaultValue = "true",
            negatable = true)
    private boolean freeAccelerationTask = true;

    /** Traffic lights task. */
    @Option(names = {"--trafficLightsTask"}, description = "Enables traffic light task.", defaultValue = "true",
            negatable = true)
    private boolean trafficLightsTask = false;

    /** Signal task. */
    @Option(names = {"--signalTask"}, description = "Enables signal task.", defaultValue = "true", negatable = true)
    private boolean signalTask = true;

    /** Lane-changing task. */
    @Option(names = {"--laneChangingTask"}, description = "Enables lane-change task.", defaultValue = "true", negatable = true)
    private boolean laneChangingTask = true;

    /** Cooperation task. */
    @Option(names = {"--cooperationTask"}, description = "Enables cooperation task.", defaultValue = "true", negatable = true)
    private boolean cooperationTask = true;

    /** Conflicts task. */
    @Option(names = {"--conflictsTask"}, description = "Enables conflict task.", defaultValue = "true", negatable = true)
    private boolean conflictsTask = true;

    /** Conflict model. */
    @Option(names = {"--conflictsInfra"}, description = "Selects infra-based conflict task.", defaultValue = "false",
            negatable = true)
    private boolean conflictsInfra = false;

    // Adaptations

    /** Behavioral adaptation of speed. */
    @Option(names = {"--speedAdaptation"}, description = "Enables speed adaptation.", defaultValue = "true", negatable = true)
    private boolean speedAdaptation = true;

    /** Behavioral adaptation of desired headway. */
    @Option(names = {"--headwayAdaptation"}, description = "Enables headway adaptation.", defaultValue = "true",
            negatable = true)
    private boolean headwayAdaptation = true;

    /** Behavioral adaptation of update time. */
    @Option(names = {"--updateTimeAdaptation"}, description = "Enables update time adaptation.", defaultValue = "true",
            negatable = true)
    private boolean updateTimeAdaptation = true;

    // Social interactions

    /** Tailgating as part of social interactions (without tailgating, social interactions still result in social pressure). */
    @Option(names = {"--tailgating"},
            description = "Enables tailgating. Without tailgating, social interactions still result in social pressure.",
            defaultValue = "true", negatable = true)
    private boolean tailgating = true;

    /** Change lane as response to social pressure. */
    @Option(names = {"--socioLaneChange"}, description = "Enables lane changes due to social pressure.", defaultValue = "true",
            negatable = true)
    private boolean socioLaneChange = true;

    /** Increase speed as response to social pressure. */
    @Option(names = {"--socioSpeed"}, description = "Enables speed increase due to social pressure.", defaultValue = "true",
            negatable = true)
    private boolean socioSpeed = true;

    // Car-following model

    /** Car-following model. */
    @Option(names = {"--carFollowing"}, description = "Car-following model: IDM, IDM_PLUS or M_IDM.", defaultValue = "M_IDM")
    private CarFollowing carFollowing = CarFollowing.M_IDM;

    /** Multi-leader anticipation. */
    @Option(names = {"--nLeaders"}, description = "Number of leaders in spatial anticipation, 1, 2 or 3.", defaultValue = "3")
    private int nLeaders = 3;

    /** Fraction of drivers overestimation as perception errors. */
    @Option(names = {"--fractionOverEstimation"},
            description = "Fraction of drivers over-estimating speed and distance [0..1].", defaultValue = "1.0")
    private double fractionOverEstimation = 1.0;

    // Localized features

    /** Active mode crossing. */
    @Option(names = {"--activeMode"}, description = "Enables reaction to active mode (task demand and braking).",
            defaultValue = "true", negatable = true)
    private boolean activeMode = true;

    /** Local distraction task. */
    @Option(names = {"--localDistraction"}, description = "Enables local distraction.", defaultValue = "true", negatable = true)
    private boolean localDistraction = true;

    /** GTU error handler for collisions. */
    private final GtuErrorHandler errorHandler = new GtuErrorHandler()
    {
        /** {@inheritDoc} */
        @Override
        public void handle(final Gtu gtu, final Exception ex) throws Exception
        {
            if (ex.getCause() instanceof CollisionException)
            {
                Method method = Gtu.class.getMethod("setOperationalPlan", OperationalPlan.class);
                method.setAccessible(true);
                method.invoke(gtu, OperationalPlan.standStill(gtu, gtu.getLocation(), gtu.getSimulator().getSimulatorAbsTime(),
                        Duration.POSITIVE_INFINITY));
            }
            else
            {
                throw ex;
            }
        }
    };

    /** Visibility. */
    private Visibility visibility;

    /**
     * Sets the random number stream.
     * @param stream random number stream
     */
    public void setStream(final StreamInterface stream)
    {
        this.stream = stream;
        this.vGainDist = new ContinuousDistSpeed(new DistLogNormal(stream, 3.379, 0.4), SpeedUnit.KM_PER_HOUR);
        this.sigmaDist = new DistTriangular(stream, 0.0, 0.25, 1.0);
        this.fSpeedDist = new DistNormalTrunc(stream, 123.7 / 120.0, 0.1, 0.8, 50.0);
    }

    /**
     * The current state will be reverted after the next tactical planner has been generated. Any changes to the settings after
     * a call to this method, and before the next tactical planner has been generated, are only applied to said tactical
     * planner.
     */
    public void setSingleShotMode()
    {
        this.state = new LinkedHashMap<>();
    }

    /**
     * Resets the factory to the state when {@code setSingleShotMode()} was called.
     */
    public void resetMode()
    {
        if (this.state == null)
        {
            return;
        }
        try
        {
            for (Entry<Field, Object> entry : this.state.entrySet())
            {
                entry.getKey().set(this, entry.getValue());
            }
        }
        catch (IllegalAccessException ex)
        {
            throw new RuntimeException(ex);
        }
        this.state = null;
    }

    /**
     * Save state of given field.
     * @param fieldName field name
     */
    private void saveState(final String fieldName)
    {
        if (this.state == null)
        {
            return;
        }
        try
        {
            Field field = ScenarioTacticalPlannerFactory.class.getDeclaredField(fieldName);
            Throw.when(this.state.containsKey(field), IllegalStateException.class,
                    "Cannot set " + fieldName + " as it was already set in single-shot mode.");
            this.state.put(field, field.get(this));
        }
        catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException ex)
        {
            throw new RuntimeException(ex);
        }
    }

    @Override
    public Parameters getParameters() throws ParameterException
    {
        ParameterSet parameters = new ParameterSet();
        parameters.setDefaultParameters(LmrsUtil.class);
        parameters.setDefaultParameters(LmrsParameters.class);
        parameters.setDefaultParameters(AbstractIdm.class);
        parameters.setDefaultParameters(ConflictUtilTmp.class);
        parameters.setDefaultParameter(ParameterTypes.T0);
        parameters.setDefaultParameter(ParameterTypes.PERCEPTION);
        parameters.setDefaultParameter(ParameterTypes.LOOKBACK);
        parameters.setDefaultParameter(ParameterTypes.LOOKAHEAD);
        parameters.setDefaultParameter(ParameterTypes.VCONG);
        parameters.setDefaultParameter(ParameterTypes.LCDUR);
        if (!this.fullerImplementation.equals(FullerImplementation.NONE))
        {
            parameters.setParameter(ParameterTypes.TR, Duration.ZERO);
            parameters.setDefaultParameter(AdaptationSituationalAwareness.TR_MAX);
            parameters.setParameter(ChannelFuller.EST_FACTOR, 1.0);
            parameters.setParameter(Estimation.OVER_EST, this.stream.nextDouble() <= this.fractionOverEstimation ? 1.0 : -1.0);
            parameters.setDefaultParameter(ChannelTaskScan.TDSCAN);
            if (this.headwayAdaptation)
            {
                parameters.setDefaultParameter(AdaptationHeadway.BETA_T);
            }
            if (this.speedAdaptation)
            {
                parameters.setDefaultParameter(AdaptationSpeed.BETA_V0);
            }

            if (this.fullerImplementation.equals(FullerImplementation.ATTENTION_MATRIX))
            {
                // Attention matrix
                parameters.setDefaultParameter(Fuller.TS_CRIT);
                parameters.setDefaultParameters(ChannelFuller.class);
                parameters.setDefaultParameters(ChannelMental.class);
                if (this.updateTimeAdaptation)
                {
                    parameters.setDefaultParameter(AdaptationUpdateTime.DT_MIN);
                    parameters.setDefaultParameter(AdaptationUpdateTime.DT_MAX);
                }
            }
            else
            {
                // Summative or anticipation reliance
                parameters.setDefaultParameters(Fuller.class);
                parameters.setDefaultParameter(AdaptationSituationalAwareness.SA);
                parameters.setDefaultParameter(AdaptationSituationalAwareness.SA_MIN);
                parameters.setDefaultParameter(AdaptationSituationalAwareness.SA_MAX);
                if (this.fullerImplementation.equals(FullerImplementation.ANTICIPATION_RELIANCE))
                {
                    parameters.setDefaultParameter(TaskManagerAr.ALPHA);
                    parameters.setDefaultParameter(TaskManagerAr.BETA);
                }
            }

            if (this.carFollowingTask)
            {
                parameters.setDefaultParameter(ChannelTaskCarFollowing.HEXP);
            }
            if (this.trafficLightsTask || this.conflictsTask)
            {
                if (this.conflictsInfra)
                {
                    parameters.setDefaultParameter(ChannelTaskIntersection.TD_EGO);
                    parameters.setDefaultParameter(ChannelTaskIntersection.XEGO);
                    parameters.setDefaultParameter(ChannelTaskIntersection.TD_OTH);
                }
                else
                {
                    parameters.setDefaultParameter(ChannelTaskConflict.HEGO);
                }
                parameters.setDefaultParameter(ChannelTaskConflict.HCONF);
            }
            if (this.signalTask)
            {
                parameters.setDefaultParameter(ChannelTaskSignal.TDSIGNAL);
            }

        }
        if (this.tailgating || this.socioLaneChange || this.socioSpeed)
        {
            parameters.setDefaultParameter(Tailgating.RHO);
            // Alternate default values in case of social interactions, including distributions for the speed-leading strategy
            parameters.setParameter(LmrsParameters.VGAIN, this.vGainDist.draw());
            parameters.setParameter(LmrsParameters.SOCIO, this.sigmaDist.draw());
            if (this.tailgating)
            {
                parameters.setParameter(ParameterTypes.TMAX, Duration.instantiateSI(1.6));
            }
        }
        if (CarFollowing.M_IDM.equals(this.carFollowing))
        {
            parameters.setDefaultParameters(IdmModified.class);
        }
        if (this.nLeaders > 1)
        {
            parameters.setParameter(CarFollowingNgoduy.NLEADERS, this.nLeaders);
            parameters.setDefaultParameter(CarFollowingNgoduy.MU1);
            if (this.nLeaders == 2)
            {
                parameters.setParameter(CarFollowingNgoduy.MU2,
                        CarFollowingNgoduy.MU2.getDefaultValue() + CarFollowingNgoduy.MU3.getDefaultValue());
            }
            else
            {
                parameters.setDefaultParameter(CarFollowingNgoduy.MU2);
                parameters.setDefaultParameter(CarFollowingNgoduy.MU3);
            }
        }
        parameters.setParameter(ParameterTypes.FSPEED, this.fSpeedDist.draw());
        return parameters;
    }

    @Override
    public ScenarioTacticalPlanner create(final LaneBasedGtu gtu) throws GtuException
    {
        gtu.setErrorHandler(this.errorHandler);

        // Car-following model
        DesiredSpeedModel desiredSpeedModel =
                this.socioSpeed ? new SocioDesiredSpeed(AbstractIdm.DESIRED_SPEED) : AbstractIdm.DESIRED_SPEED;
        CarFollowingModel carFollowingModel;
        switch (this.carFollowing)
        {
            case IDM:
                carFollowingModel = new Idm(AbstractIdm.HEADWAY, desiredSpeedModel);
                break;
            case IDM_PLUS:
                carFollowingModel = new IdmPlus(AbstractIdm.HEADWAY, desiredSpeedModel);
                break;
            case M_IDM:
                carFollowingModel = new IdmModified(AbstractIdm.HEADWAY, desiredSpeedModel);
                break;
            default:
                throw new IllegalArgumentException("Unable to load car-following model from setting " + this.carFollowing);
        }
        if (this.nLeaders > 1)
        {
            carFollowingModel = new CarFollowingNgoduy(carFollowingModel);
        }

        // Perception
        LanePerception perception = getPerception(gtu);

        // Tactical planner
        Tailgating tail = this.tailgating ? Tailgating.PRESSURE
                : (this.socioLaneChange || this.socioSpeed ? Tailgating.RHO_ONLY : Tailgating.NONE);
        ScenarioTacticalPlanner tacticalPlanner = new ScenarioTacticalPlanner(carFollowingModel, gtu, perception,
                Synchronization.PASSIVE, Cooperation.PASSIVE, GapAcceptance.INFORMED, tail);
        tacticalPlanner.addMandatoryIncentive(new IncentiveRoute());
        tacticalPlanner.addVoluntaryIncentive(new IncentiveSpeedWithCourtesy());
        tacticalPlanner.addVoluntaryIncentive(new IncentiveKeep());
        tacticalPlanner.addAccelerationIncentive(new AccelerationConflictsTmp());
        if (this.activeMode && FullerImplementation.ATTENTION_MATRIX.equals(this.fullerImplementation))
        {
            tacticalPlanner.addAccelerationIncentive(new AccelerationActiveModeCrossing());
        }
        if (this.socioLaneChange)
        {
            tacticalPlanner.addVoluntaryIncentive(new IncentiveSocioSpeed());
        }
        return tacticalPlanner;
    }

    /**
     * Returns perception for the GTU.
     * @param gtu GTU
     * @return perception for the GTU
     */
    private LanePerception getPerception(final LaneBasedGtu gtu)
    {
        Mental mental;
        Estimation estimationNeighbors;
        Estimation estimationConflicts;
        Anticipation anticipationNeighbors;
        Anticipation anticipationConflicts;

        if (FullerImplementation.ATTENTION_MATRIX.equals(this.fullerImplementation))
        {
            LinkedHashSet<Function<LanePerception, Set<ChannelTask>>> taskSuppliers = new LinkedHashSet<>();
            addTask(taskSuppliers, this.carFollowingTask, ChannelTaskCarFollowing.SUPPLIER);
            addTask(taskSuppliers, this.freeAccelerationTask, ChannelTaskAcceleration.SUPPLIER);
            addTask(taskSuppliers, this.trafficLightsTask, ChannelTaskTrafficLight.SUPPLIER);
            addTask(taskSuppliers, this.signalTask, ChannelTaskSignal.SUPPLIER);
            addTask(taskSuppliers, this.laneChangingTask, ChannelTaskLaneChange.SUPPLIER);
            addTask(taskSuppliers, this.cooperationTask, ChannelTaskCooperation.SUPPLIER);
            addTask(taskSuppliers, this.conflictsTask,
                    this.conflictsInfra ? ChannelTaskIntersection.SUPPLIER : ChannelTaskConflict.SUPPLIER);
            addTask(taskSuppliers, this.activeMode, ChannelTaskActiveModeCrossing.SUPPLIER);
            addTask(taskSuppliers, this.localDistraction, ChannelTaskLocalDistraction.SUPPLIER);
            addTask(taskSuppliers, true, ChannelTaskScan.SUPPLIER);

            Set<BehavioralAdaptation> behavioralAdapatations = new LinkedHashSet<>();
            if (this.speedAdaptation)
            {
                behavioralAdapatations.add(new AdaptationSpeedChannel());
            }
            if (this.headwayAdaptation)
            {
                behavioralAdapatations.add(new AdaptationHeadwayChannel());
            }
            if (this.updateTimeAdaptation)
            {
                behavioralAdapatations.add(new AdaptationUpdateTime());
            }

            mental = new ChannelFuller(taskSuppliers, behavioralAdapatations);

            estimationNeighbors = new SaturationEstimation(true);
            estimationConflicts = new SaturationEstimation(false);
            anticipationNeighbors = this.temporalAnticipation ? Anticipation.CONSTANT_SPEED : Anticipation.NONE;
            anticipationConflicts = new ConflictAnticipation();
        }
        else if (FullerImplementation.NONE.equals(this.fullerImplementation))
        {
            mental = null;
            estimationNeighbors = Estimation.NONE;
            estimationConflicts = Estimation.NONE;
            anticipationNeighbors = Anticipation.NONE;
            anticipationConflicts = Anticipation.NONE;
        }
        else
        {
            TaskManager taskManager;
            if (FullerImplementation.SUMMATIVE.equals(this.fullerImplementation))
            {
                taskManager = new SummativeTaskManager();
            }
            else if (FullerImplementation.ANTICIPATION_RELIANCE.equals(this.fullerImplementation))
            {
                taskManager = new TaskManagerAr(this.primaryTask.getPrimaryTaskId());
            }
            else
            {
                throw new IllegalArgumentException("Unable to load Fuller model from setting " + this.fullerImplementation);
            }

            Set<Task> tasks = new LinkedHashSet<>();
            if (this.carFollowingTask || this.primaryTask.equals(PrimaryTask.CAR_FOLLOWING))
            {
                tasks.add(new CarFollowingTask());
            }
            if (this.laneChangingTask || this.primaryTask.equals(PrimaryTask.LANE_CHANGING))
            {
                tasks.add(new LaneChangeTask());
            }
            if (this.localDistraction)
            {
                tasks.add(new LocalDistractionTask());
            }

            Set<BehavioralAdaptation> behavioralAdapatations = new LinkedHashSet<>();
            if (this.speedAdaptation)
            {
                behavioralAdapatations.add(new AdaptationSpeed());
            }
            if (this.headwayAdaptation)
            {
                behavioralAdapatations.add(new AdaptationHeadway());
            }
            behavioralAdapatations.add(new AdaptationSituationalAwareness());

            mental = new Fuller(tasks, behavioralAdapatations, taskManager);

            estimationNeighbors = Estimation.FACTOR_ESTIMATION;
            estimationConflicts = Estimation.FACTOR_ESTIMATION;
            anticipationNeighbors = this.temporalAnticipation ? Anticipation.CONSTANT_SPEED : Anticipation.NONE;
            anticipationConflicts = new ConflictAnticipation();
        }

        LanePerception perception = new VisibilityLanePerception(gtu, mental, this.visibility);
        perception.addPerceptionCategory(new DirectEgoPerception<>(perception));
        perception.addPerceptionCategory(new DirectInfrastructurePerception(perception));
        perception.addPerceptionCategory(new AnticipationTrafficPerception(perception));
        if (this.localDistraction)
        {
            perception.addPerceptionCategory(new LocalDistractionPerception(perception));
        }
        if (FullerImplementation.NONE.equals(this.fullerImplementation))
        {
            perception.addPerceptionCategory(new DirectNeighborsPerception(perception, HeadwayGtuType.WRAP));
            perception.addPerceptionCategory(new DirectIntersectionPerception(perception, HeadwayGtuType.WRAP));
        }
        else if (FullerImplementation.ATTENTION_MATRIX.equals(this.fullerImplementation))
        {
            if (this.activeMode)
            {
                perception.addPerceptionCategory(new ActiveModePerception(perception));
            }
            perception.addPerceptionCategory(
                    new NeighborsPerceptionChannel(perception, estimationNeighbors, anticipationNeighbors));
            perception.addPerceptionCategory(
                    new IntersectionPerceptionChannel(perception, estimationConflicts, anticipationConflicts));
        }
        else
        {
            HeadwayGtuType headwayGtuTypeNeighbors = new PerceivedHeadwayGtuType(estimationNeighbors, anticipationNeighbors);
            perception.addPerceptionCategory(new DirectNeighborsPerception(perception, headwayGtuTypeNeighbors));
            HeadwayGtuType headwayGtuTypeConflicts = new PerceivedHeadwayGtuType(estimationConflicts, anticipationConflicts);
            perception.addPerceptionCategory(new DirectIntersectionPerception(perception, headwayGtuTypeConflicts));
        }
        return perception;
    }

    /**
     * Add task if required.
     * @param taskSuppliers suppliers to add task suppliers to
     * @param task whether to add task
     * @param supplier supplier for the task
     */
    private void addTask(final LinkedHashSet<Function<LanePerception, Set<ChannelTask>>> taskSuppliers, final boolean task,
            final Function<LanePerception, Set<ChannelTask>> supplier)
    {
        if (task)
        {
            taskSuppliers.add(supplier);
        }
    }

    /**
     * Sets the Fuller implementation.
     * @param fullerImplementation Fuller implementation
     */
    public void setFullerImplementation(final FullerImplementation fullerImplementation)
    {
        Throw.whenNull(fullerImplementation, "fullerImplementation");
        saveState("fullerImplementation");
        this.fullerImplementation = fullerImplementation;
    }

    /**
     * Sets the primary task under {@code FullerImplementation.ANTICIPATION_RELIANCE}.
     * @param primaryTask primary task under {@code FullerImplementation.ANTICIPATION_RELIANCE}
     */
    public void setPrimaryTask(final PrimaryTask primaryTask)
    {
        Throw.whenNull(primaryTask, "primaryTask");
        saveState("primaryTask");
        this.primaryTask = primaryTask;
    }

    /**
     * Sets the temporal anticipation, which follows a constant-speed heuristic.
     * @param temporalAnticipation anticipation
     */
    public void setTemporalAnticipation(final boolean temporalAnticipation)
    {
        Throw.whenNull(temporalAnticipation, "temporalAnticipation");
        saveState("temporalAnticipation");
        this.temporalAnticipation = temporalAnticipation;
    }

    /**
     * Enables/disables car-following task.
     * @param carFollowingTask enable car-following task
     */
    public void setCarFollowingTask(final boolean carFollowingTask)
    {
        saveState("carFollowingTask");
        this.carFollowingTask = carFollowingTask;
    }

    /**
     * Enables/disables free acceleration task ({@code FullerImplementation.ATTENTION_MATRIX} only).
     * @param freeAccelerationTask enable free acceleration task
     */
    public void setFreeAccelerationTask(final boolean freeAccelerationTask)
    {
        saveState("freeAccelerationTask");
        this.freeAccelerationTask = freeAccelerationTask;
    }

    /**
     * Enables/disables traffic lights task ({@code FullerImplementation.ATTENTION_MATRIX} only).
     * @param trafficLightsTask enable traffic lights task
     */
    public void setTrafficLightsTask(final boolean trafficLightsTask)
    {
        saveState("trafficLightsTask");
        this.trafficLightsTask = trafficLightsTask;
    }

    /**
     * Enables/disables signal task ({@code FullerImplementation.ATTENTION_MATRIX} only).
     * @param signalTask enable signal task
     */
    public void setSignalTask(final boolean signalTask)
    {
        saveState("signalTask");
        this.signalTask = signalTask;
    }

    /**
     * Enables/disables lane-changing task.
     * @param laneChangingTask enable lane-changing task
     */
    public void setLaneChangingTask(final boolean laneChangingTask)
    {
        saveState("laneChangingTask");
        this.laneChangingTask = laneChangingTask;
    }

    /**
     * Enables/disables cooperation task ({@code FullerImplementation.ATTENTION_MATRIX} only).
     * @param cooperationTask enable cooperation task
     */
    public void setCooperationTask(final boolean cooperationTask)
    {
        saveState("cooperationTask");
        this.cooperationTask = cooperationTask;
    }

    /**
     * Enables/disables conflicts task ({@code FullerImplementation.ATTENTION_MATRIX} only).
     * @param conflictsTask enable conflicts task
     */
    public void setConflictsTask(final boolean conflictsTask)
    {
        saveState("conflictsTask");
        this.conflictsTask = conflictsTask;
    }

    /**
     * Sets the conflicts task demand model infra-based or not.
     * @param conflictsInfra conflicts task demand model infra-based or not
     */
    public void setConflictsInfra(final boolean conflictsInfra)
    {
        saveState("conflictsInfra");
        this.conflictsInfra = conflictsInfra;
    }

    /**
     * Enables/disables behavioral adaptation of speed.
     * @param speedAdaptation behavioral adaptation of speed
     */
    public void setSpeedAdaptation(final boolean speedAdaptation)
    {
        saveState("speedAdaptation");
        this.speedAdaptation = speedAdaptation;
    }

    /**
     * Enables/disables behavioral adaptation of desired headway.
     * @param headwayAdaptation behavioral adaptation of desired headway
     */
    public void setHeadwayAdaptation(final boolean headwayAdaptation)
    {
        saveState("headwayAdaptation");
        this.headwayAdaptation = headwayAdaptation;
    }

    /**
     * Enables/disables behavioral adaptation of update time ({@code FullerImplementation.ATTENTION_MATRIX} only).
     * @param updateTimeAdaptation behavioral adaptation of update time
     */
    public void setUpdateTimeAdaptation(final boolean updateTimeAdaptation)
    {
        saveState("updateTimeAdaptation");
        this.updateTimeAdaptation = updateTimeAdaptation;
    }

    /**
     * Enables/disables tailgating.
     * @param tailgating tailgating
     */
    public void setTailgating(final boolean tailgating)
    {
        saveState("tailgating");
        this.tailgating = tailgating;
    }

    /**
     * Enables/disables social lane changes.
     * @param socioLaneChange socioLaneChange
     */
    public void setSocioLaneChange(final boolean socioLaneChange)
    {
        saveState("socioLaneChange");
        this.socioLaneChange = socioLaneChange;
    }

    /**
     * Enables/disables social speed increase.
     * @param socioSpeed socioSpeed
     */
    public void setSocioSpeed(final boolean socioSpeed)
    {
        saveState("socioSpeed");
        this.socioSpeed = socioSpeed;
    }

    /**
     * Sets the car-following model.
     * @param carFollowing car-following model
     */
    public void setCarFollowing(final CarFollowing carFollowing)
    {
        Throw.whenNull(carFollowing, "carFollowing");
        saveState("carFollowing");
        this.carFollowing = carFollowing;
    }

    /**
     * Sets the number of anticipated leaders for car-following.
     * @param numberOfLeaders number of anticipated leaders for car-following
     */
    public void setNumberOfLeaders(final int numberOfLeaders)
    {
        Throw.when(numberOfLeaders < 1, IllegalArgumentException.class, "Number of leaders should be at least 1.");
        Throw.when(numberOfLeaders > 3, IllegalArgumentException.class, "Number of leaders should be at most 3.");
        saveState("nLeaders");
        this.nLeaders = numberOfLeaders;
    }

    /**
     * Sets fraction of drivers overestimating perception errors.
     * @param fractionOverEstimation fraction of drivers overestimating perception errors
     */
    public void setFractionOverEstimation(final double fractionOverEstimation)
    {
        Throw.when(fractionOverEstimation < 0.0, IllegalArgumentException.class,
                "Fraction overestimation should be above or equal to 0.0");
        Throw.when(fractionOverEstimation > 1.0, IllegalArgumentException.class,
                "Fraction overestimation should be below or equal to 1.0");
        saveState("fractionOverEstimation");
        this.fractionOverEstimation = fractionOverEstimation;
    }

    /**
     * Enables/disables active mode crossing task ({@code FullerImplementation.ATTENTION_MATRIX} only).
     * @param activeMode enable active mode task
     */
    public void setActiveMode(final boolean activeMode)
    {
        saveState("activeMode");
        this.activeMode = activeMode;
    }

    /**
     * Enables/disables local distraction.
     * @param localDistraction enable local distraction
     */
    public void setLocalDistraction(final boolean localDistraction)
    {
        saveState("localDistraction");
        this.localDistraction = localDistraction;
    }

    /**
     * Set visibility.
     * @param visibility
     */
    public void setVisibility(final Visibility visibility)
    {
        this.visibility = visibility;
    }

    /**
     * Type of management of different tasks.
     */
    public enum FullerImplementation
    {
        /** No perception model. */
        NONE,

        /** Task demands are summed. */
        SUMMATIVE,

        /**
         * Task demand based on one primary and multiple auxiliary tasks. Requires parameters ALPHA and BETA and the id of the
         * primary task.
         */
        ANTICIPATION_RELIANCE,

        /** Task demand as steady-state in the attention matrix. */
        ATTENTION_MATRIX;
    }

    /**
     * Defines what task is primary under {@code FullerImplementation.ANTICIPATION_RELIANCE}.
     */
    public enum PrimaryTask
    {
        /** Lane-changing. */
        LANE_CHANGING("lane-changing"),

        /** Car-following. */
        CAR_FOLLOWING("car-following");

        /** Id of primary task. */
        private final String primaryTaskId;

        /**
         * Constructor.
         * @param primaryTaskId id of primary task
         */
        PrimaryTask(final String primaryTaskId)
        {
            this.primaryTaskId = primaryTaskId;
        }

        /**
         * Returns the id of the primary task.
         * @return id of the primary task.
         */
        public String getPrimaryTaskId()
        {
            return this.primaryTaskId;
        }
    }

    /**
     * Car-following model.
     */
    public enum CarFollowing
    {
        /** IDM. */
        IDM,

        /** IDM+. */
        IDM_PLUS,

        /** M-IDM. */
        M_IDM;
    }

}
