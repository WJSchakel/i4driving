package org.opentrafficsim.i4driving.profiles;

import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import org.djunits.unit.SpeedUnit;
import org.djunits.value.base.Scalar;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djutils.cli.CliUtil;
import org.djutils.data.Column;
import org.djutils.data.ListTable;
import org.djutils.data.csv.CsvData;
import org.djutils.draw.line.PolyLine2d;
import org.djutils.draw.line.Polygon2d;
import org.djutils.draw.point.OrientedPoint2d;
import org.djutils.exceptions.Throw;
import org.opentrafficsim.base.Resource;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterType;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.definitions.Defaults;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.geometry.FractionalLengthData;
import org.opentrafficsim.core.geometry.OtsGeometryException;
import org.opentrafficsim.core.geometry.OtsLine2d;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.i4driving.sim0mq.GtuSpawnerOd;
import org.opentrafficsim.i4driving.sim0mq.ParameterFactorySim0mq;
import org.opentrafficsim.i4driving.tactical.ScenarioTacticalPlanner;
import org.opentrafficsim.i4driving.tactical.ScenarioTacticalPlannerFactory;
import org.opentrafficsim.i4driving.tactical.ScenarioTacticalPlannerFactory.CarFollowing;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelFuller;
import org.opentrafficsim.road.definitions.DefaultsRoadNl;
import org.opentrafficsim.road.gtu.generator.characteristics.DefaultLaneBasedGtuCharacteristicsGeneratorOd;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationHeadway;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSpeed;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.tactical.following.AbstractIdm;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalRoutePlannerFactory;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.CrossSectionSlice;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.LaneGeometryUtil;
import org.opentrafficsim.road.network.lane.changing.LaneKeepingPolicy;
import org.opentrafficsim.swing.script.AbstractSimulationScript;

import de.siegmar.fastcsv.reader.CsvReader;
import de.siegmar.fastcsv.reader.CsvRow;
import nl.tudelft.simulation.jstats.streams.MersenneTwister;
import picocli.CommandLine.Command;
import picocli.CommandLine.Mixin;
import picocli.CommandLine.Option;

/**
 * This class runs an experiment based on UNINA calibration data (profiles/profiles.csv in resources). It runs a harsh
 * deceleration simulation for each profile, ranks them by safety indicators, and outputs indicators and parameter values.
 * @author wjschakel
 */
@Command(description = "Scenario in which people are teste for their safety level by their parameters.", name = "Car-following",
        mixinStandardHelpOptions = true, showDefaultValues = true)
public class CarFollowingScenario extends AbstractSimulationScript
{

    /** */
    private static final long serialVersionUID = 1L;

    /** List of parsed vehicle IDs. */
    private static List<String> vehicleIds;

    /** Parsed parameter values per vehicle ID. */
    private static Map<String, List<String>> values = new LinkedHashMap<>();

    /** Model factory. */
    @Mixin
    private ScenarioTacticalPlannerFactory mixinModel = new ScenarioTacticalPlannerFactory();

    /** Speed limit in experiment. */
    @Option(names = "--speedLimit", description = "Speed limit", defaultValue = "50km/h")
    private Speed speedLimit = new Speed(50.0, SpeedUnit.KM_PER_HOUR);

    /** Leader deceleration in experiment. */
    @Option(names = "--leaderDeceleration", description = "Deceleration of the leader", defaultValue = "9.0m/s2")
    private Acceleration deceleration = Acceleration.instantiateSI(9.0);

    /** Vehicle ID of ego vehicle in data. */
    @Option(names = "--vehicleId", description = "Vehicle ID")
    private String vehicleId;

    /** Start time of leader deceleration. */
    @Option(names = "--tDecelerate", description = "Start time of leader deceleration.", defaultValue = "120s")
    private Duration tDecelerate = Duration.instantiateSI(120.0);

    /** Length of the network. */
    @Option(names = "--networkLength", description = "Length of the network.", defaultValue = "2500m")
    private Length networkLength = Length.instantiateSI(2500.0);

    /** Time between statistic checks. */
    @Option(names = "--statisticTime", description = "Time between statistic checks.", defaultValue = "0.01s")
    private Duration statisticTime = Duration.instantiateSI(0.01);

    /** Whether to run with IDM+, or M-IDM otherwise. */
    @Option(names = "--idm", description = "Run with IDM+.", defaultValue = "false")
    private boolean idmPlus = false;

    /** Number of statistic checks performed. */
    private long statisticChecks = 0;

    /** Minimum gap during simulation. */
    private Length minGap = Length.POSITIVE_INFINITY;

    /** Minimum TTC during simulation. */
    private Duration minTtc = Duration.POSITIVE_INFINITY;

    /** Statistic of safety indicators from simulation. */
    private Statistic statistic;

    /** GTU spawner. */
    private GtuSpawnerOd spawner;

    /** Parameter factory. */
    private ParameterFactorySim0mq parameterFactory;

    /** Output values of a single run. */
    private Map<String, Object> tableValues = new LinkedHashMap<>();

    /**
     * Runs a simulation for each profile and saves output.
     * @param args command line arguments
     * @throws Exception exception
     */
    public static void main(String[] args) throws Exception
    {
        CliUtil.changeOptionDefault(CarFollowingScenario.class, "autorun", "true");

        // Read profiles as String values
        try (CsvReader csv = CsvReader.builder().fieldSeparator(';')
                .build(new InputStreamReader(Resource.getResourceAsStream("/profiles/profiles.csv"))))
        {
            boolean processHeader = true;
            for (CsvRow row : csv)
            {
                List<String> valueList = new ArrayList<>();
                row.getFields().forEach((v) -> valueList.add(v.toString().trim()));
                "﻿vehicle_ID".equals(valueList.get(0));
                if (processHeader)
                {
                    Throw.when(!"﻿vehicle_ID".equals(valueList.get(0)), IllegalArgumentException.class,
                            "The first row is not labelled vehicle_ID.");
                    vehicleIds = valueList.subList(1, valueList.size());
                    processHeader = false;
                }
                else
                {
                    values.put(valueList.get(0),
                            valueList.subList(1, valueList.size()).stream().map((v) -> v.replace(",", ".").trim()).toList());
                }
            }
        }

        // Run simulation for each profile
        Map<Statistic, Set<Map<String, Object>>> statistics = new TreeMap<>();
        boolean idmPlus = false;
        for (String vehicleId : vehicleIds)
        {
            // if (!"14".equals(vehicleId))
            // {
            // continue;
            // }
            if (vehicleId.trim().isEmpty())
            {
                continue; // empty columns from Excel files saved as profiles.csv
            }
            CarFollowingScenario scenario = new CarFollowingScenario(vehicleId);
            CliUtil.execute(scenario, args);
            idmPlus = scenario.idmPlus;
            try
            {
                scenario.start();
            }
            catch (UnsupportedOperationException ex)
            {
                // Expected simulation end (avoiding System.exit)
                scenario.tableValues.put("vehicleId", vehicleId);
                scenario.tableValues.put("minTtc", scenario.statistic.minTtc);
                scenario.tableValues.put("dv", scenario.statistic.dv);
                scenario.tableValues.put("minGap", scenario.statistic.minGap);
                statistics.computeIfAbsent(scenario.statistic, (v) -> new LinkedHashSet<>())
                        .add(new LinkedHashMap<>(scenario.tableValues));
            }
            catch (RuntimeException ex)
            {
                // Simulation ran in to an unexpected exception
                System.out.println("Exception for " + vehicleId);
                scenario.tableValues.put("vehicleId", vehicleId);
                scenario.tableValues.put("minTtc", null);
                scenario.tableValues.put("dv", null);
                scenario.tableValues.put("minGap", null);
                scenario.tableValues.put("Tbrake", null);
                statistics.computeIfAbsent(new Statistic(null, null, null), (v) -> new LinkedHashSet<>())
                        .add(scenario.tableValues);
            }

        }

        // Create, fill and write table.
        ListTable table = new ListTable("results", "results table",
                List.of(new Column<>("vehicleId", "ID of vehicle in UNINA dataset", String.class),
                        new Column<>("minTtc", "Minimium time-to-collision", Duration.class, "s"),
                        new Column<>("dv", "Speed difference at collision", Speed.class, "m/s"),
                        new Column<>("minGap", "Minimum gap during deceleration", Length.class, "m"),
                        new Column<>("Tbrake", "Time headway when braking starts", Duration.class, "s"),
                        columnFor(ParameterTypes.A), columnFor(ParameterTypes.B), columnFor(ParameterTypes.BCRIT),
                        columnFor(ParameterTypes.S0), columnFor(ParameterTypes.TMIN), columnFor(ParameterTypes.TMAX),
                        columnFor(ParameterTypes.FSPEED), columnFor(AbstractIdm.DELTA), columnFor(ParameterTypes.TAU),
                        columnFor(LmrsParameters.VGAIN), columnFor(LmrsParameters.SOCIO), columnFor(AdaptationHeadway.BETA_T),
                        columnFor(AdaptationSpeed.BETA_V0),
                        // columnFor(CarFollowingTask.HEXP),columnFor(ChannelTaskLaneChange.TD_D),
                        // columnFor(ChannelTaskSignal.TDSIGNAL),columnFor(ChannelTaskScan.TDSCAN),
                        columnFor(Fuller.TC), columnFor(Fuller.TS_CRIT),
                        new Column<>("x0", ParameterTypes.LOOKAHEAD.getDescription(), ParameterTypes.LOOKAHEAD.getValueClass(),
                                ParameterTypes.LOOKAHEAD.getDefaultValue().getDisplayUnit().getStandardUnit().getId()),
                        columnFor(ParameterTypes.T0), columnFor(ChannelFuller.TAU_MIN), columnFor(ChannelFuller.TAU_MAX),
                        columnFor(ParameterTypes.VCONG)));
        statistics.values().forEach((s) -> s.forEach((m) -> table.addRowByColumnIds(m)));
        String model = idmPlus ? "idm_plus" : "m-idm";
        CsvData.writeData("profiles_" + model + ".csv", "profiles_" + model + "_meta.csv", table);
    }

    /**
     * Returns a column for a parameter type.
     * @param <T> value type
     * @param parameterType parameter type
     * @return column for parameter type
     * @throws ParameterException if the parameter type has a unit but no default value
     */
    private static <T> Column<T> columnFor(final ParameterType<T> parameterType) throws ParameterException
    {
        if (Scalar.class.isAssignableFrom(parameterType.getValueClass()))
        {
            Scalar<?, ?> s = (Scalar<?, ?>) parameterType.getDefaultValue();
            return new Column<>(parameterType.getId(), parameterType.getDescription(), parameterType.getValueClass(),
                    s.getDisplayUnit().getStandardUnit().getId());
        }
        return new Column<>(parameterType.getId(), parameterType.getDescription(), parameterType.getValueClass());
    }

    /**
     * Constructor.
     * @param vehicleId vehicle ID to run with
     */
    public CarFollowingScenario(final String vehicleId)
    {
        super("Car-following", "Scenario in which people are teste for their safety level by their parameters.");
        Throw.whenNull(vehicleId, "vehicleId");
        Throw.when(!vehicleIds.contains(vehicleId), IllegalArgumentException.class, "Vehicle with ID %s is not known.",
                vehicleId);
        this.vehicleId = vehicleId;
    }

    @Override
    protected RoadNetwork setupSimulation(final OtsSimulatorInterface sim) throws Exception
    {
        // Network
        RoadNetwork network = new RoadNetwork("car-following", sim);
        Node nodeA = new Node(network, "A", new OrientedPoint2d(0.0, 0.0, 0.0));
        Node nodeB = new Node(network, "B", new OrientedPoint2d(this.networkLength.si, 0.0, 0.0));
        OtsLine2d line = new OtsLine2d(nodeA.getPoint(), nodeB.getPoint());
        FractionalLengthData elevation = FractionalLengthData.of(0.0, 0.0, 0.0, 0.0);
        CrossSectionLink ab = new CrossSectionLink(network, "AB", nodeA, nodeB, DefaultsNl.URBAN, line, elevation,
                LaneKeepingPolicy.KEEPRIGHT);
        PolyLine2d left = line.offsetLine(1.75).getLine2d();
        PolyLine2d right = line.offsetLine(-1.75).getLine2d();
        Polygon2d contour = LaneGeometryUtil.getContour(left, right);
        List<CrossSectionSlice> elements = List.of(new CrossSectionSlice(Length.ZERO, Length.ZERO, Length.instantiateSI(3.5)),
                new CrossSectionSlice(Length.instantiateSI(this.networkLength.si), Length.ZERO, Length.instantiateSI(3.5)));
        Lane lane =
                new Lane(ab, "lane", line, contour, elements, DefaultsRoadNl.URBAN_ROAD, Map.of(DefaultsNl.CAR, speedLimit));

        // Model setup
        Route route = new Route("AB", DefaultsNl.CAR, List.of(nodeA, nodeB));
        this.parameterFactory = new ParameterFactorySim0mq();
        this.spawner = new GtuSpawnerOd(network, new DefaultLaneBasedGtuCharacteristicsGeneratorOd.Factory(
                new LaneBasedStrategicalRoutePlannerFactory(this.mixinModel, this.parameterFactory)).create());
        GtuType.registerTemplateSupplier(DefaultsNl.CAR, Defaults.NL);
        this.mixinModel.setStream(new MersenneTwister(getSeed()));
        if (this.idmPlus)
        {
            this.mixinModel.setCarFollowing(CarFollowing.IDM_PLUS);
        }

        // Vehicles
        leadingVehicle(sim, lane, route);
        followingVehicle(sim, lane, route);
        egoVehicle(sim, lane, route);

        return network;
    }

    /**
     * Create leading vehicle
     * @param sim simulator
     * @param lane lane
     * @param route route
     * @throws IllegalArgumentException exception
     * @throws GtuException exception
     * @throws OtsGeometryException exception
     * @throws NetworkException exception
     */
    private void leadingVehicle(final OtsSimulatorInterface sim, final Lane lane, final Route route)
            throws IllegalArgumentException, GtuException, OtsGeometryException, NetworkException
    {
        this.mixinModel.setSingleShotMode();
        /*
         * The leading vehicle get default values, except FSPEED. In this way we exclude randomness in the leader speed. We set
         * 1.0 for the value so the leader drives at the speed limit. Note that this is done knowing that all calibrated profile
         * values for this value have FSPEED > 1.0, meaning that the ego vehicle will be car-following.
         */
        this.parameterFactory.setParameterValue(ParameterTypes.FSPEED, 1.0);
        this.spawner.spawnGtu("leader", DefaultsNl.CAR, Length.instantiateSI(4.0), Length.instantiateSI(2.0),
                Length.instantiateSI(3.0), route, this.speedLimit, new OrientedPoint2d(210.0, 0.0, 0.0));
        this.parameterFactory.clearSetParameters();
        sim.scheduleEventAbs(Duration.instantiateSI(120.0), () -> decelerateLeader());
    }

    /**
     * Scheduled start of harsh deceleration by the leader.
     */
    private void decelerateLeader()
    {
        ((ScenarioTacticalPlanner) getNetwork().getGTU("leader").getTacticalPlanner()).setAcceleration(deceleration.neg());
    }

    /**
     * Create ego vehicle
     * @param sim simulator
     * @param lane lane
     * @param route route
     * @throws IllegalArgumentException exception
     * @throws GtuException exception
     * @throws OtsGeometryException exception
     * @throws NetworkException exception
     */
    private void egoVehicle(final OtsSimulatorInterface sim, final Lane lane, final Route route)
            throws IllegalArgumentException, GtuException, OtsGeometryException, NetworkException, ParameterException
    {
        this.mixinModel.setSingleShotMode();
        /*
         * For the ego vehicle all parameters values from a profile that can be set are set.
         */
        int vehicleIdIndex = vehicleIds.indexOf(this.vehicleId);
        Double v0 = null;
        Double vCritPercLC = null;
        Double tauMin_front = null, tauMin_left = null, tauMin_right = null, tauMin_back = null, tauDelta = null;
        for (String parameterId : values.keySet())
        {
            String valueString = values.get(parameterId).get(vehicleIdIndex);
            switch (parameterId)
            {
                case "a":
                    setParameter(ParameterTypes.A, Acceleration.valueOf(valueString + "m/s2"));
                    break;
                case "b":
                    Acceleration b = Acceleration.valueOf(valueString + "m/s2");
                    if (b.si >= ParameterTypes.BCRIT.getDefaultValue().si)
                    {
                        setParameter(ParameterTypes.BCRIT, b.plus(Acceleration.instantiateSI(0.01)));
                    }
                    else
                    {
                        setParameter(ParameterTypes.BCRIT, ParameterTypes.BCRIT.getDefaultValue());
                    }
                    setParameter(ParameterTypes.B, b);
                    break;
                case "s0":
                    setParameter(ParameterTypes.S0, Length.valueOf(valueString + "m"));
                    break;
                case "T":
                    setParameter(ParameterTypes.TMIN, Duration.valueOf(valueString + "s"));
                    break;
                case "Tmax":
                    setParameter(ParameterTypes.TMAX, Duration.valueOf(valueString + "s"));
                    break;
                case "v0":
                    v0 = Double.valueOf(valueString);
                    setParameter(ParameterTypes.FSPEED, v0 / (130.0 / 3.6));
                    break;
                case "delta":
                    setParameter(AbstractIdm.DELTA, Double.valueOf(valueString));
                    break;
                case "lambda_relax":
                    setParameter(ParameterTypes.TAU, Duration.valueOf(valueString + "s"));
                    break;
                case "vGain":
                    setParameter(LmrsParameters.VGAIN, Speed.valueOf(valueString + "m/s"));
                    break;
                case "vCritPerc_LC":
                    vCritPercLC = Double.valueOf(valueString);
                    break;
                case "sigma":
                    setParameter(LmrsParameters.SOCIO, Double.valueOf(valueString));
                    break;
                case "beta_T":
                    setParameter(AdaptationHeadway.BETA_T, Double.valueOf(valueString));
                    break;
                case "beta_v0":
                    setParameter(AdaptationSpeed.BETA_V0, Double.valueOf(valueString));
                    break;
                /*
                 * The below four parameters are excluded as task demand values create excessive task saturation, which even
                 * when compared to larger than 1 TScrit values, result in unreasonable behavioral adaptation, i.e. huge
                 * headways.
                 */
                case "wTD_CF":
                    // setParameter(CarFollowingTask.HEXP, Duration.valueOf(valueString + "s"));
                    break;
                case "wTD_LC":
                    // setParameter(ChannelTaskLaneChange.TD_D, Double.valueOf(valueString));
                    break;
                case "wTD_indicator":
                    // setParameter(ChannelTaskSignal.TDSIGNAL, Double.valueOf(valueString));
                    break;
                case "wTD_scan":
                    // setParameter(ChannelTaskScan.TDSCAN, Double.valueOf(valueString));
                    break;
                case "TC":
                    setParameter(Fuller.TC, Double.valueOf(valueString));
                    break;
                case "TScrit":
                    setParameter(Fuller.TS_CRIT, Double.valueOf(valueString));
                    break;
                case "tpTDmax":
                    tauDelta = Double.valueOf(valueString);
                    break;
                case "tpFixed_front":
                    tauMin_front = Double.valueOf(valueString);
                    break;
                case "tpFixed_left":
                    tauMin_left = Double.valueOf(valueString);
                    break;
                case "tpFixed_right":
                    tauMin_right = Double.valueOf(valueString);
                    break;
                case "tpFixed_back":
                    tauMin_back = Double.valueOf(valueString);
                    break;
                case "x0_front":
                    // ParameterTypes.LOOKAHEAD has id "Look-ahead" but we need "x0", hence do this not by setParameter()
                    Length value = Length.valueOf(valueString + "m");
                    this.tableValues.put("x0", value);
                    this.parameterFactory.setParameterValue(ParameterTypes.LOOKAHEAD, value);
                    break;
                case "t0":
                    setParameter(ParameterTypes.T0, Duration.valueOf(valueString + "s"));
                    break;
                case "deltaTmax": // Tmax is in the parameter directly
                case "dFree": // single-lane experiment, plus there are issue with desire normalization in the calibration
                case "dSyncPerc": // id.
                case "lambda_LC": // OTS gap-acceptance makes no distinction between ego and follower decelerations
                case "lambda_rho": // OTS has no passive social pressure
                case "lambda_tailgating": // id.
                case "beta_d": // OTS has no behavioral adaptation on lane change desire
                case "wTD_ndrt_alert": // no NDRT in the experiment
                case "wTD_ndrt_actionPerc": // id.
                case "lambda_TS": // perception errors were excluded from calibration
                case "epsilon_v": // id.
                case "epsilon_s_front": // id.
                case "epsilon_s_back": // id.
                case "epsilon_dv_front": // id.
                case "epsilon_dv_back": // id.
                case "epsilon_a_back": // id.
                case "x0_back": // OTS makes no distinction between front and rear perception
                case "y0_in": // OTS does not need to look at lateral distance, lane bookkeeping determines leader
                case "y0_out":
                case "":
                    // cases above purposefully ignored
                    break;
                default:
                    throw new IllegalArgumentException("Unable to parse parameter " + parameterId);
            }
        }
        Throw.when(tauMin_front == null || tauMin_left == null, tauMin_right == null || tauMin_back == null,
                IllegalArgumentException.class,
                "Unable to set tau_min as tauMin_front, tauMin_left, tauMin_right or tauMin_back is null.");
        Duration tauMin = Duration.instantiateSI((tauMin_front + tauMin_left + tauMin_right + tauMin_back) / 4.0);
        setParameter(ChannelFuller.TAU_MIN, tauMin);
        Throw.when(tauDelta == null, IllegalArgumentException.class, "Unable to set tau_max as tauDelta is null.");
        setParameter(ChannelFuller.TAU_MAX, tauMin.plus(Duration.instantiateSI(tauDelta)));
        Throw.when(v0 == null || vCritPercLC == null, IllegalArgumentException.class,
                "Unable to set vCong as v0 or vCritPercLC is null.");
        setParameter(ParameterTypes.VCONG, Speed.instantiateSI(vCritPercLC * v0));
        this.spawner.spawnGtu("ego", DefaultsNl.CAR, Length.instantiateSI(4.0), Length.instantiateSI(2.0),
                Length.instantiateSI(3.0), route, this.speedLimit, new OrientedPoint2d(110.0, 0.0, 0.0));
        this.parameterFactory.clearSetParameters();
        sim.scheduleEventAbs(this.tDecelerate, () -> checkStatistic(true));
    }

    /**
     * Sets a parameter both in the output table values and parameter factory.
     * @param <T> value type
     * @param parameterType parameter type
     * @param value value
     */
    private <T> void setParameter(ParameterType<T> parameterType, T value)
    {
        this.tableValues.put(parameterType.getId(), value);
        this.parameterFactory.setParameterValue(parameterType, value);
    }

    /**
     * Scheduled check on statistics.
     * @param sampleTimeHeadway whether to sample the headway value (true at first invocation)
     */
    private void checkStatistic(boolean sampleTimeHeadway)
    {
        try
        {
            // Get positions and speeds
            LaneBasedGtu leader = (LaneBasedGtu) getNetwork().getGTU("leader");
            LaneBasedGtu ego = (LaneBasedGtu) getNetwork().getGTU("ego");
            Length leaderRear = leader.position(leader.getReferencePosition().lane(), leader.getRear());
            Length egoFront = ego.position(ego.getReferencePosition().lane(), ego.getFront());
            Length gap = leaderRear.minus(egoFront);
            Speed egoSpeed = ego.getSpeed();
            Speed leaderSpeed = leader.getSpeed();

            // Store headway at start of deceleration
            if (sampleTimeHeadway)
            {
                this.tableValues.put("Tbrake", gap.divide(egoSpeed));
            }

            // TTC
            Duration ttc = null;
            if (egoSpeed.gt(leaderSpeed))
            {
                ttc = Duration.instantiateSI(gap.si / (egoSpeed.si - leaderSpeed.si));
                this.minTtc = Duration.min(this.minTtc, ttc);
            }

            // Check collision
            if (gap.lt0())
            {
                this.statistic = new Statistic(Length.ZERO, egoSpeed.minus(leaderSpeed), Duration.ZERO);
                getNetwork().getSimulator().endReplication();
                // mainThread.interrupt();
                return;
            }
            this.minGap = Length.min(this.minGap, gap);

            // Check stand-still
            if (egoSpeed.eq0())
            {
                this.statistic = new Statistic(this.minGap, null, this.minTtc);
                getNetwork().getSimulator().endReplication();
                // mainThread.interrupt();
                return;
            }

            // Schedule next check
            this.statisticChecks++;
            Duration tNextCheck = this.tDecelerate.plus(this.statisticTime.times(this.statisticChecks));
            getNetwork().getSimulator().scheduleEventAbs(tNextCheck, () -> checkStatistic(false));
        }
        catch (GtuException ex)
        {
            throw new RuntimeException(ex);
        }
    }

    /**
     * Create following vehicle
     * @param sim simulator
     * @param lane lane
     * @param route route
     * @throws IllegalArgumentException exception
     * @throws GtuException exception
     * @throws OtsGeometryException exception
     * @throws NetworkException exception
     */
    private void followingVehicle(final OtsSimulatorInterface sim, final Lane lane, final Route route)
            throws IllegalArgumentException, GtuException, OtsGeometryException, NetworkException
    {
        this.mixinModel.setSingleShotMode();
        /*
         * The following vehicle gets default values, except FSPEED and VGAIN. These two values are set such that social
         * pressure (called tailgating in the experiment description) is present, and the vehicle will be in car-following mode.
         */
        this.parameterFactory.setParameterValue(ParameterTypes.FSPEED, 1.25);
        this.parameterFactory.setParameterValue(LmrsParameters.VGAIN, new Speed(25.0, SpeedUnit.KM_PER_HOUR));
        this.spawner.spawnGtu("follower", DefaultsNl.CAR, Length.instantiateSI(4.0), Length.instantiateSI(2.0),
                Length.instantiateSI(3.0), route, this.speedLimit, new OrientedPoint2d(10.0, 0.0, 0.0));
        this.parameterFactory.clearSetParameters();
        sim.scheduleEventAbs(this.tDecelerate, () -> decelerateLeader());
    }

    /**
     * Avoid System.exit in super class, which kills the java runtime.
     */
    protected void onSimulationEnd()
    {
        throw new UnsupportedOperationException();
    }

    /**
     * Statistic class.
     * @param minGap minimum gap during deceleration
     * @param dv speed difference at collision (or {@code null} if no collision)
     * @param minTtc minimum TTC during deceleration
     */
    private record Statistic(Length minGap, Speed dv, Duration minTtc) implements Comparable<Statistic>
    {
        @Override
        public int compareTo(final Statistic o)
        {
            int cmp = -compare(this.minTtc, o.minTtc);
            if (cmp != 0)
            {
                return cmp;
            }
            return compare(this.dv, o.dv);
        }

        /**
         * Compare individual value
         * @param <T> value type
         * @param t value of this Statistic
         * @param o value of other Statistic
         * @return
         */
        private <T> int compare(Comparable<T> t, T o)
        {
            if (t == null)
            {
                if (o == null)
                {
                    return 0;
                }
                return -1;
            }
            if (o == null)
            {
                return 1;
            }
            return t.compareTo(o);
        }

        @Override
        public String toString()
        {
            if (this.dv == null)
            {
                if (this.minTtc == null)
                {
                    return "null";
                }
                return this.minTtc.toString();
            }
            return this.dv.toString();
        }
    }

}
