package org.opentrafficsim.i4driving.demo;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Predicate;

import org.djunits.unit.FrequencyUnit;
import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Frequency;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.vector.FrequencyVector;
import org.djunits.value.vdouble.vector.TimeVector;
import org.djutils.cli.CliUtil;
import org.djutils.draw.line.PolyLine2d;
import org.djutils.draw.line.Polygon2d;
import org.djutils.draw.point.OrientedPoint2d;
import org.djutils.draw.point.Point2d;
import org.opentrafficsim.animation.colorer.FixedColor;
import org.opentrafficsim.animation.gtu.colorer.AccelerationGtuColorer;
import org.opentrafficsim.animation.gtu.colorer.GtuColorer;
import org.opentrafficsim.animation.gtu.colorer.SpeedGtuColorer;
import org.opentrafficsim.animation.gtu.colorer.SwitchableGtuColorer;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterSet;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.definitions.Defaults;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.geometry.Bezier;
import org.opentrafficsim.core.geometry.ContinuousBezierCubic;
import org.opentrafficsim.core.geometry.ContinuousLine;
import org.opentrafficsim.core.geometry.ContinuousStraight;
import org.opentrafficsim.core.geometry.Flattener;
import org.opentrafficsim.core.geometry.Flattener.NumSegments;
import org.opentrafficsim.core.geometry.FractionalLengthData;
import org.opentrafficsim.core.geometry.OtsLine2d;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;
import org.opentrafficsim.core.perception.HistoryManagerDevs;
import org.opentrafficsim.draw.graphs.ContourDataSource;
import org.opentrafficsim.draw.graphs.GraphPath;
import org.opentrafficsim.draw.graphs.GraphPath.Section;
import org.opentrafficsim.i4driving.demo.plots.ContourPlotExtendedData;
import org.opentrafficsim.i4driving.demo.plots.DistributionPlotExtendedData;
import org.opentrafficsim.i4driving.sampling.TaskSaturationData;
import org.opentrafficsim.i4driving.tactical.perception.ChannelPerceptionFactory;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelFuller;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelMental;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTask;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ConflictUtilTmp;
import org.opentrafficsim.kpi.sampling.data.ExtendedDataNumber;
import org.opentrafficsim.road.definitions.DefaultsRoadNl;
import org.opentrafficsim.road.gtu.generator.characteristics.DefaultLaneBasedGtuCharacteristicsGeneratorOd;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionFactory;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Estimation;
import org.opentrafficsim.road.gtu.lane.tactical.AbstractLaneBasedTacticalPlannerFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.CarFollowingModelFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlus;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlusFactory;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationTrafficLights;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveRoute;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.Lmrs;
import org.opentrafficsim.road.gtu.lane.tactical.util.TrafficLightUtil;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Cooperation;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.GapAcceptance;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsUtil;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Synchronization;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Tailgating;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalRoutePlannerFactory;
import org.opentrafficsim.road.network.LaneAccessLaw;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.CrossSectionLink.Priority;
import org.opentrafficsim.road.network.lane.CrossSectionSlice;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.LaneGeometryUtil;
import org.opentrafficsim.road.network.lane.LaneType;
import org.opentrafficsim.road.network.lane.Shoulder;
import org.opentrafficsim.road.network.lane.Stripe;
import org.opentrafficsim.road.network.lane.Stripe.Type;
import org.opentrafficsim.road.network.lane.changing.LaneKeepingPolicy;
import org.opentrafficsim.road.network.lane.conflict.Conflict;
import org.opentrafficsim.road.network.lane.conflict.ConflictBuilder;
import org.opentrafficsim.road.network.lane.conflict.ConflictBuilder.RelativeWidthGenerator;
import org.opentrafficsim.road.network.lane.object.trafficlight.TrafficLight;
import org.opentrafficsim.road.network.sampling.GtuDataRoad;
import org.opentrafficsim.road.network.sampling.LaneDataRoad;
import org.opentrafficsim.road.network.sampling.RoadSampler;
import org.opentrafficsim.road.network.sampling.data.TimeToCollision;
import org.opentrafficsim.road.od.Categorization;
import org.opentrafficsim.road.od.Category;
import org.opentrafficsim.road.od.Interpolation;
import org.opentrafficsim.road.od.OdApplier;
import org.opentrafficsim.road.od.OdMatrix;
import org.opentrafficsim.road.od.OdOptions;
import org.opentrafficsim.swing.graphs.SwingContourPlot;
import org.opentrafficsim.swing.graphs.SwingPlot;
import org.opentrafficsim.swing.gui.OtsSimulationApplication;
import org.opentrafficsim.swing.script.AbstractSimulationScript;
import org.opentrafficsim.trafficcontrol.FixedTimeController;
import org.opentrafficsim.trafficcontrol.FixedTimeController.SignalGroup;

import nl.tudelft.simulation.dsol.swing.gui.TablePanel;
import nl.tudelft.simulation.jstats.distributions.DistContinuous;
import nl.tudelft.simulation.jstats.streams.StreamInterface;

/**
 * Demo of attention in an urban setting.
 * @author wjschakel
 */
public class AttentionDemoUrban extends AbstractSimulationScript
{

    /** */
    private static final long serialVersionUID = 20240926L;

    /** Shoulder lane type. */
    private static final LaneType SHOULDER = new LaneType("Shoulder");

    /** Front attention data type. */
    private static final DataTypeAttention DATA_ATT_FRONT =
            new DataTypeAttention(ChannelTask.FRONT.toString(), (c) -> ChannelTask.FRONT.equals(c));

    /** Conflicts attention data type. */
    private static final DataTypeAttention DATA_ATT_CONFLICTS =
            new DataTypeAttention("conflicts", (c) -> c instanceof Conflict);

    /** Task saturation data type. */
    private static final TaskSaturationData DATA_SATURATION = new TaskSaturationData();

    /** Time-to-collision data type. */
    private static final TimeToCollision DATA_TTC = new TimeToCollision();

    /** Link length. */
    private final double linkLength = 100.0;

    /** Intersection radius. */
    private final double intersection = 15.0;

    /**
     * Constructor.
     */
    protected AttentionDemoUrban()
    {
        super("Attention urban", "Demo of attention in an urban setting.");
        @SuppressWarnings("deprecation")
        GtuColorer colorer = new SwitchableGtuColorer(0, new FixedColor(Color.BLUE, "Blue"),
                new SpeedGtuColorer(new Speed(60.0, SpeedUnit.KM_PER_HOUR)),
                new AccelerationGtuColorer(Acceleration.instantiateSI(-6.0), Acceleration.instantiateSI(2.0)),
                new SplitColorer(), new TaskSaturationChannelColorer());
        setGtuColorer(colorer);
    }

    /**
     * Main program.
     * @param args command line arguments. See AbstractSimulationScript for available arguments.
     * @throws Exception when an exception occurs.
     */
    public static void main(final String[] args) throws Exception
    {
        AttentionDemoUrban demo = new AttentionDemoUrban();
        CliUtil.execute(demo, args);
        demo.start();
    }

    @SuppressWarnings({"checkstyle:methodlength", "checkstyle:firstsentence"})
    /** {@inhertiDoc} */
    @Override
    protected final RoadNetwork setupSimulation(final OtsSimulatorInterface sim) throws Exception
    {
        RoadNetwork network = new RoadNetwork("urban demo", sim);
        sim.getReplication()
                .setHistoryManager(new HistoryManagerDevs(sim, Duration.instantiateSI(5.0), Duration.instantiateSI(10.0)));

        // Eastern intersection
        OrientedPoint2d pointNin2 = new OrientedPoint2d(this.linkLength / 2.0 + this.intersection / 3.0,
                this.linkLength + this.intersection / 2.0, Math.PI * 1.5);
        OrientedPoint2d pointNout2 = new OrientedPoint2d(this.linkLength / 2.0 + 2.0 * this.intersection / 3.0,
                this.linkLength + this.intersection / 2.0, Math.PI * 0.5);
        OrientedPoint2d pointSin2 = new OrientedPoint2d(this.linkLength / 2.0 + 2.0 * this.intersection / 3.0,
                -this.linkLength - this.intersection / 2.0, Math.PI * 0.5);
        OrientedPoint2d pointSout2 = new OrientedPoint2d(this.linkLength / 2.0 + this.intersection / 3.0,
                -this.linkLength - this.intersection / 2.0, Math.PI * 1.5);
        OrientedPoint2d pointPNin2 =
                new OrientedPoint2d(this.linkLength / 2.0 + this.intersection / 3.0, this.intersection / 2.0, Math.PI * 1.5);
        OrientedPoint2d pointPNout2 = new OrientedPoint2d(this.linkLength / 2.0 + 2.0 * this.intersection / 3.0,
                this.intersection / 2.0, Math.PI * 0.5);
        OrientedPoint2d pointPSin2 = new OrientedPoint2d(this.linkLength / 2.0 + 2.0 * this.intersection / 3.0,
                -this.intersection / 2.0, Math.PI * 0.5);
        OrientedPoint2d pointPSout2 =
                new OrientedPoint2d(this.linkLength / 2.0 + this.intersection / 3.0, -this.intersection / 2.0, Math.PI * 1.5);
        Node nodeNin2 = new Node(network, "Nin2", pointNin2);
        Node nodeNout2 = new Node(network, "Nout2", pointNout2);
        Node nodeSin2 = new Node(network, "Sin2", pointSin2);
        Node nodeSout2 = new Node(network, "Sout2", pointSout2);
        Node nodePNin2 = new Node(network, "PNin2", pointPNin2);
        Node nodePNout2 = new Node(network, "PNout2", pointPNout2);
        Node nodePSin2 = new Node(network, "PSin2", pointPSin2);
        Node nodePSout2 = new Node(network, "PSout2", pointPSout2);

        OrientedPoint2d pointEin =
                new OrientedPoint2d(this.linkLength * 1.5 + this.intersection, this.intersection / 6.0, Math.PI * 1.0);
        OrientedPoint2d pointEout =
                new OrientedPoint2d(this.linkLength * 1.5 + this.intersection, -this.intersection / 6.0, 0.0);
        OrientedPoint2d pointPEin =
                new OrientedPoint2d(this.linkLength * 0.5 + this.intersection, this.intersection / 6.0, Math.PI * 1.0);
        OrientedPoint2d pointPEout =
                new OrientedPoint2d(this.linkLength * 0.5 + this.intersection, -this.intersection / 6.0, 0.0);
        OrientedPoint2d pointCEin = new OrientedPoint2d(this.linkLength * 0.5, this.intersection / 6.0, Math.PI * 1.0);
        OrientedPoint2d pointCEout = new OrientedPoint2d(this.linkLength * 0.5, -this.intersection / 6.0, 0.0);
        Node nodeEin = new Node(network, "Ein", pointEin);
        Node nodeEout = new Node(network, "Eout", pointEout);
        Node nodePEin = new Node(network, "PEin", pointPEin);
        Node nodePEout = new Node(network, "PEout", pointPEout);
        Node nodeCEin = new Node(network, "CEin", pointCEin);
        Node nodeCEout = new Node(network, "CEout", pointCEout);

        // Western intersection
        double dx = -this.linkLength - this.intersection;
        OrientedPoint2d pointNin1 = new OrientedPoint2d(dx + this.linkLength / 2.0 + this.intersection / 3.0,
                this.linkLength + this.intersection / 2.0, Math.PI * 1.5);
        OrientedPoint2d pointNout1 = new OrientedPoint2d(dx + this.linkLength / 2.0 + 2.0 * this.intersection / 3.0,
                this.linkLength + this.intersection / 2.0, Math.PI * 0.5);
        OrientedPoint2d pointSin1 = new OrientedPoint2d(dx + this.linkLength / 2.0 + 2.0 * this.intersection / 3.0,
                -this.linkLength - this.intersection / 2.0, Math.PI * 0.5);
        OrientedPoint2d pointSout1 = new OrientedPoint2d(dx + this.linkLength / 2.0 + this.intersection / 3.0,
                -this.linkLength - this.intersection / 2.0, Math.PI * 1.5);
        OrientedPoint2d pointPNin1 = new OrientedPoint2d(dx + this.linkLength / 2.0 + this.intersection / 3.0,
                this.intersection / 2.0, Math.PI * 1.5);
        OrientedPoint2d pointPNout1 = new OrientedPoint2d(dx + this.linkLength / 2.0 + 2.0 * this.intersection / 3.0,
                this.intersection / 2.0, Math.PI * 0.5);
        OrientedPoint2d pointPSin1 = new OrientedPoint2d(dx + this.linkLength / 2.0 + 2.0 * this.intersection / 3.0,
                -this.intersection / 2.0, Math.PI * 0.5);
        OrientedPoint2d pointPSout1 = new OrientedPoint2d(dx + this.linkLength / 2.0 + this.intersection / 3.0,
                -this.intersection / 2.0, Math.PI * 1.5);
        Node nodeNin1 = new Node(network, "Nin1", pointNin1);
        Node nodeNout1 = new Node(network, "Nout1", pointNout1);
        Node nodeSin1 = new Node(network, "Sin1", pointSin1);
        Node nodeSout1 = new Node(network, "Sout1", pointSout1);
        Node nodePNin1 = new Node(network, "PNin1", pointPNin1);
        Node nodePNout1 = new Node(network, "PNout1", pointPNout1);
        Node nodePSin1 = new Node(network, "PSin1", pointPSin1);
        Node nodePSout1 = new Node(network, "PSout1", pointPSout1);

        OrientedPoint2d pointWout =
                new OrientedPoint2d(-this.linkLength * 1.5 - this.intersection, this.intersection / 6.0, Math.PI * 1.0);
        OrientedPoint2d pointWin =
                new OrientedPoint2d(-this.linkLength * 1.5 - this.intersection, -this.intersection / 6.0, 0.0);
        OrientedPoint2d pointPWout =
                new OrientedPoint2d(-this.linkLength * 0.5 - this.intersection, this.intersection / 6.0, Math.PI * 1.0);
        OrientedPoint2d pointPWin =
                new OrientedPoint2d(-this.linkLength * 0.5 - this.intersection, -this.intersection / 6.0, 0.0);
        OrientedPoint2d pointCWout = new OrientedPoint2d(-this.linkLength * 0.5, this.intersection / 6.0, Math.PI * 1.0);
        OrientedPoint2d pointCWin = new OrientedPoint2d(-this.linkLength * 0.5, -this.intersection / 6.0, 0.0);
        Node nodeWout = new Node(network, "Wout", pointWout);
        Node nodeWin = new Node(network, "Win", pointWin);
        Node nodePWin = new Node(network, "PWin", pointPWin);
        Node nodePWout = new Node(network, "PWout", pointPWout);
        Node nodeCWin = new Node(network, "CWin", pointCWin);
        Node nodeCWout = new Node(network, "CWout", pointCWout);

        // Links
        makeLink(network, nodeNin2, nodePNin2);
        makeLink(network, nodePNout2, nodeNout2);
        makeLink(network, nodeEin, nodePEin);
        makeLink(network, nodePEout, nodeEout);
        makeLink(network, nodeSin2, nodePSin2);
        makeLink(network, nodePSout2, nodeSout2);
        makeLink(network, nodePNin2, nodeCEin);
        makeLink(network, nodePNin2, nodePSout2);
        makeLink(network, nodePNin2, nodePEout);
        makeLink(network, nodePEin, nodePNout2);
        makeLink(network, nodePEin, nodeCEin);
        makeLink(network, nodePEin, nodePSout2);
        makeLink(network, nodePSin2, nodePEout);
        makeLink(network, nodePSin2, nodePNout2);
        makeLink(network, nodePSin2, nodeCEin);
        makeLink(network, nodeCEout, nodePSout2);
        makeLink(network, nodeCEout, nodePEout);
        makeLink(network, nodeCEout, nodePNout2);

        makeLink(network, nodeCEin, nodeCWout);
        makeLink(network, nodeCWin, nodeCEout);

        makeLink(network, nodeNin1, nodePNin1);
        makeLink(network, nodePNout1, nodeNout1);
        makeLink(network, nodePWout, nodeWout);
        makeLink(network, nodeWin, nodePWin);
        makeLink(network, nodeSin1, nodePSin1);
        makeLink(network, nodePSout1, nodeSout1);
        makeLink(network, nodePNin1, nodePWout);
        makeLink(network, nodePNin1, nodePSout1);
        makeLink(network, nodePNin1, nodeCWin);
        makeLink(network, nodeCWout, nodePNout1);
        makeLink(network, nodeCWout, nodePWout);
        makeLink(network, nodeCWout, nodePSout1);
        makeLink(network, nodePSin1, nodeCWin);
        makeLink(network, nodePSin1, nodePNout1);
        makeLink(network, nodePSin1, nodePWout);
        makeLink(network, nodePWin, nodePSout1);
        makeLink(network, nodePWin, nodeCWin);
        makeLink(network, nodePWin, nodePNout1);

        List<Node> origins = new ArrayList<>();
        List<Node> destinations = new ArrayList<>();
        for (Node node : network.getNodeMap().values())
        {
            if (node.getId().startsWith("N") || node.getId().startsWith("E") || node.getId().startsWith("S")
                    || node.getId().startsWith("W"))
            {
                if (node.getId().contains("in"))
                {
                    origins.add(node);
                }
                else if (node.getId().contains("out"))
                {
                    destinations.add(node);
                }
            }
        }
        OdMatrix od = new OdMatrix("od", origins, destinations, Categorization.UNCATEGORIZED,
                new TimeVector(new double[] {0.0, 3600.0}), Interpolation.LINEAR);
        for (Node from : origins)
        {
            for (Node to : destinations)
            {
                // Skip U-turns
                if (!from.getId().replace("in", "out").equals(to.getId()))
                {
                    boolean main = (from.getId().startsWith("E") || from.getId().startsWith("W"))
                            && (to.getId().startsWith("E") || to.getId().startsWith("W"));
                    double f = 1.0;
                    double[] demand = main ? new double[] {200.0 * f, 300.0 * f} : new double[] {20.0 * f, 30.0 * f};
                    od.putDemandVector(from, to, Category.UNCATEGORIZED, new FrequencyVector(demand, FrequencyUnit.PER_HOUR));
                }
            }
        }

        StreamInterface stream = sim.getModel().getStream("generation");
        ParameterFactoryByType parameterFactory = new ParameterFactoryByType();
        double fractionUnder = 0.8;
        double error = 0.4;
        parameterFactory.addParameter(Estimation.OVER_EST, new DistContinuous(stream)
        {
            /** */
            private static final long serialVersionUID = 20240930L;

            /** {@inheritDoc} */
            @Override
            public double getProbabilityDensity(final double x)
            {
                return x == -error ? fractionUnder : (x == error ? error - fractionUnder : error);
            }

            /** {@inheritDoc} */
            @Override
            public double draw()
            {
                return getStream().nextDouble() <= fractionUnder ? -error : error;
            }
        });
        PerceptionFactory perceptionFactory = new ChannelPerceptionFactory();
        OdOptions options = new OdOptions();
        CarFollowingModelFactory<IdmPlus> cfFactory = new IdmPlusFactory(stream);
        AbstractLaneBasedTacticalPlannerFactory<Lmrs> lmrsFactory =
                new AbstractLaneBasedTacticalPlannerFactory<>(cfFactory, perceptionFactory)
                {
                    /** {@inheritDoc} */
                    @Override
                    public Lmrs create(final LaneBasedGtu gtu) throws GtuException
                    {
                        Lmrs lmrs = new Lmrs(nextCarFollowingModel(gtu), gtu, getPerceptionFactory().generatePerception(gtu),
                                Synchronization.ALIGN_GAP, Cooperation.PASSIVE, GapAcceptance.INFORMED, Tailgating.NONE);
                        lmrs.addMandatoryIncentive(new IncentiveRoute());
                        lmrs.addAccelerationIncentive(new AccelerationConflictsTmp());
                        lmrs.addAccelerationIncentive(new AccelerationTrafficLights());
                        return lmrs;
                    }

                    /** {@inheritDoc} */
                    @Override
                    public Parameters getParameters() throws ParameterException
                    {
                        ParameterSet parameters = new ParameterSet();
                        parameters.setDefaultParameters(LmrsUtil.class);
                        parameters.setDefaultParameters(LmrsParameters.class);
                        parameters.setDefaultParameters(ConflictUtilTmp.class);
                        parameters.setDefaultParameters(TrafficLightUtil.class);
                        getCarFollowingParameters().setAllIn(parameters);
                        getPerceptionFactory().getParameters().setAllIn(parameters);
                        parameters.setDefaultParameter(ParameterTypes.VCONG);
                        parameters.setDefaultParameter(ParameterTypes.T0);
                        parameters.setDefaultParameter(ChannelMental.X0_D);
                        parameters.setDefaultParameter(ParameterTypes.LCDUR);
                        return parameters;
                    }

                };
        options.set(OdOptions.GTU_TYPE, new DefaultLaneBasedGtuCharacteristicsGeneratorOd.Factory(
                new LaneBasedStrategicalRoutePlannerFactory(lmrsFactory, parameterFactory)).create());
        GtuType.registerTemplateSupplier(DefaultsNl.CAR, Defaults.NL);
        OdApplier.applyOd(network, od, options, DefaultsRoadNl.VEHICLES);

        // TODO: When OTS issue #134 is published, we can use ConflictBuilder.DEFAULT_WIDTH_GENERATOR
        ConflictBuilder.buildConflicts(network, sim, new RelativeWidthGenerator(0.7));

        Lane lane = ((CrossSectionLink) network.getLink("Sin2_PSin2")).getLanes().get(0);
        Route route = new Route("route", DefaultsNl.CAR, List.of(nodeSin2, nodePSin2, nodePNout2, nodeNout2));
        network.getLaneChangeInfo(lane, route, DefaultsNl.CAR, Length.instantiateSI(2000.0), LaneAccessLaw.LEGAL);

        addTrafficLights(network);

        return network;
    }

    /**
     * Make link.
     * @param network network
     * @param from from node
     * @param to to node
     * @throws NetworkException when cross-section element input is not correct
     */
    private void makeLink(final RoadNetwork network, final Node from, final Node to) throws NetworkException
    {
        ContinuousLine line;
        double distance = from.getPoint().distance(to.getPoint());
        Flattener flattener = new NumSegments(64);
        if (from.getHeading().eq(to.getHeading()))
        {
            line = new ContinuousStraight(from.getLocation(), from.getLocation().distance(to.getLocation()));
        }
        else
        {
            double shape = distance < this.intersection / 2.0 ? 1.0 : 0.55;
            Point2d[] designPoints = Bezier.cubicControlPoints(from.getLocation(), to.getLocation(), shape, false);
            line = new ContinuousBezierCubic(designPoints[0], designPoints[1], designPoints[2], designPoints[3]);
        }
        OtsLine2d designLine = new OtsLine2d(line.flattenOffset(new FractionalLengthData(0.0, 0.0), flattener));
        CrossSectionLink link = new CrossSectionLink(network, from.getId() + "_" + to.getId(), from, to, DefaultsNl.URBAN,
                designLine, new FractionalLengthData(0.0, 0.0), LaneKeepingPolicy.KEEPRIGHT);
        if (from.getId().contains("E") || from.getId().contains("W"))
        {
            link.setPriority(Priority.PRIORITY);
        }

        designLine = new OtsLine2d(line.flattenOffset(new FractionalLengthData(0.0, 3.5), flattener));
        List<CrossSectionSlice> slices = LaneGeometryUtil.getSlices(line, Length.instantiateSI(3.5), Length.instantiateSI(3.5));
        PolyLine2d left = line.flattenOffset(new FractionalLengthData(0.0, 1.75), flattener);
        PolyLine2d right = line.flattenOffset(new FractionalLengthData(0.0, 5.25), flattener);
        Polygon2d contour = LaneGeometryUtil.getContour(left, right);
        new Shoulder(link, "leftShoulder", designLine, contour, slices, SHOULDER);

        designLine = new OtsLine2d(line.flattenOffset(new FractionalLengthData(0.0, -3.5), flattener));
        slices = LaneGeometryUtil.getSlices(line, Length.instantiateSI(-3.5), Length.instantiateSI(3.5));
        left = line.flattenOffset(new FractionalLengthData(0.0, -1.75), flattener);
        right = line.flattenOffset(new FractionalLengthData(0.0, -5.25), flattener);
        contour = LaneGeometryUtil.getContour(left, right);
        new Shoulder(link, "rightShoulder", designLine, contour, slices, SHOULDER);

        designLine = new OtsLine2d(line.flattenOffset(new FractionalLengthData(0.0, 0.0), flattener));
        slices = LaneGeometryUtil.getSlices(line, Length.instantiateSI(0.0), Length.instantiateSI(3.5));
        left = line.flattenOffset(new FractionalLengthData(0.0, -1.75), flattener);
        right = line.flattenOffset(new FractionalLengthData(0.0, 1.75), flattener);
        contour = LaneGeometryUtil.getContour(left, right);
        new Lane(link, "lane", designLine, contour, slices, DefaultsRoadNl.URBAN_ROAD,
                Map.of(DefaultsNl.VEHICLE, new Speed(50.0, SpeedUnit.KM_PER_HOUR)));

        if (distance > this.linkLength / 2.0)
        {
            designLine = new OtsLine2d(line.flattenOffset(new FractionalLengthData(0.0, 1.75), flattener));
            slices = LaneGeometryUtil.getSlices(line, Length.instantiateSI(0.0), Length.instantiateSI(0.2));
            left = line.flattenOffset(new FractionalLengthData(0.0, 1.85), flattener);
            right = line.flattenOffset(new FractionalLengthData(0.0, 1.65), flattener);
            contour = LaneGeometryUtil.getContour(left, right);
            new Stripe(Type.SOLID, link, designLine, contour, slices);
        }
        if (distance < this.intersection / 2.0 || distance > this.linkLength / 2.0)
        {
            designLine = new OtsLine2d(line.flattenOffset(new FractionalLengthData(0.0, -1.75), flattener));
            slices = LaneGeometryUtil.getSlices(line, Length.instantiateSI(0.0), Length.instantiateSI(0.2));
            left = line.flattenOffset(new FractionalLengthData(0.0, -1.85), flattener);
            right = line.flattenOffset(new FractionalLengthData(0.0, -1.65), flattener);
            contour = LaneGeometryUtil.getContour(left, right);
            new Stripe(Type.SOLID, link, designLine, contour, slices);
        }
    }

    /**
     * Add traffic lights.
     * @param network network
     * @throws NetworkException when traffic light input is not correct
     */
    private void addTrafficLights(final RoadNetwork network) throws NetworkException
    {
        Lane north = ((CrossSectionLink) network.getLink("Nin1_PNin1")).getLanes().get(0);
        Lane east = ((CrossSectionLink) network.getLink("CEin_CWout")).getLanes().get(0);
        Lane south = ((CrossSectionLink) network.getLink("Sin1_PSin1")).getLanes().get(0);
        Lane west = ((CrossSectionLink) network.getLink("Win_PWin")).getLanes().get(0);

        Length dx = Length.instantiateSI(2.0);
        new TrafficLight("light", north, north.getLength().minus(dx), network.getSimulator());
        new TrafficLight("light", east, east.getLength().minus(dx), network.getSimulator());
        new TrafficLight("light", south, south.getLength().minus(dx), network.getSimulator());
        new TrafficLight("light", west, west.getLength().minus(dx), network.getSimulator());

        Set<SignalGroup> groups = new LinkedHashSet<>();
        Duration yellow = Duration.instantiateSI(3.0);
        groups.add(new SignalGroup("north", Set.of("Nin1_PNin1.lane.light", "Sin1_PSin1.lane.light"),
                Duration.instantiateSI(0.0), Duration.instantiateSI(17.0), yellow));
        groups.add(new SignalGroup("east", Set.of("CEin_CWout.lane.light"), Duration.instantiateSI(20.0),
                Duration.instantiateSI(32.0), yellow));
        groups.add(new SignalGroup("west", Set.of("Win_PWin.lane.light"), Duration.instantiateSI(55.0),
                Duration.instantiateSI(32.0), yellow));
        new FixedTimeController("controller", network.getSimulator(), network, Duration.instantiateSI(90.0), Duration.ZERO,
                groups);
    }

    @Override
    protected void addTabs(final OtsSimulatorInterface sim, final OtsSimulationApplication<?> animation)
    {

        RoadSampler sampler = new RoadSampler(Set.of(DATA_ATT_FRONT, DATA_ATT_CONFLICTS, DATA_SATURATION, DATA_TTC),
                Collections.emptySet(), getNetwork(), Frequency.instantiateSI(2.0));
        addPlots("West-East", List.of("Win", "PWin", "CWin", "CEout", "PEout", "Eout"), 0, sim, animation, sampler);
        addPlots("East-West", List.of("Ein", "PEin", "CEin", "CWout", "PWout", "Wout"), 0, sim, animation, sampler);
        addPlots("North-South (West)", List.of("Nin1", "PNin1", "PSout1", "Sout1"), 0, sim, animation, sampler);
        addPlots("South-North (West)", List.of("Sin1", "PSin1", "PNout1", "Nout1"), 0, sim, animation, sampler);
        addPlots("North-South (East)", List.of("Nin2", "PNin2", "PSout2", "Sout2"), 0, sim, animation, sampler);
        addPlots("South-North (East)", List.of("Sin2", "PSin2", "PNout2", "Nout2"), 0, sim, animation, sampler);
    }

    /**
     * Adds plots for a single path, defined by a list of nodes, and lane number on the links.
     * @param tabName tab name
     * @param nodes list of nodes
     * @param laneNum lane number on link
     * @param sim simulator
     * @param animation animation
     * @param sampler sampler
     */
    private void addPlots(final String tabName, final List<String> nodes, final int laneNum, final OtsSimulatorInterface sim,
            final OtsSimulationApplication<?> animation, final RoadSampler sampler)
    {
        GraphPath<LaneDataRoad> graphPath = new GraphPath<>(tabName, getSections(nodes, laneNum));
        GraphPath.initRecording(sampler, graphPath);
        TablePanel charts = new TablePanel(2, 2);
        ContourDataSource source = new ContourDataSource(sampler.getSamplerData(), graphPath);
        ContourPlotExtendedData front =
                new ContourPlotExtendedData("Attention front", sim, source, DATA_ATT_FRONT, 0.0, 1.0, 0.2);
        charts.setCell(new SwingContourPlot(front).getContentPane(), 0, 0);
        ContourPlotExtendedData conflicts =
                new ContourPlotExtendedData("Attention conflicts (sum)", sim, source, DATA_ATT_CONFLICTS, 0.0, 1.0, 0.2);
        charts.setCell(new SwingContourPlot(conflicts).getContentPane(), 1, 0);
        ContourPlotExtendedData saturation =
                new ContourPlotExtendedData("Task saturation", sim, source, DATA_SATURATION, 0.0, 3.0, 0.5);
        charts.setCell(new SwingContourPlot(saturation).getContentPane(), 0, 1);
        DistributionPlotExtendedData ttc = new DistributionPlotExtendedData(sampler.getSamplerData(), graphPath, DATA_TTC,
                "Time-to-collision", "Time-to-collision [s]", sim, 0.0, 0.5, 8.0);
        charts.setCell(new SwingPlot(ttc).getContentPane(), 1, 1);
        animation.getAnimationPanel().getTabbedPane().addTab(animation.getAnimationPanel().getTabbedPane().getTabCount(),
                tabName, charts);
    }

    /**
     * Get list of graph path sections.
     * @param nodes link names
     * @param laneNum lane number
     * @return list of graph path sections
     */
    private List<Section<LaneDataRoad>> getSections(final List<String> nodes, final int laneNum)
    {
        List<Section<LaneDataRoad>> out = new ArrayList<>();
        for (int i = 0; i < nodes.size() - 1; i++)
        {
            String linkId = nodes.get(i) + "_" + nodes.get(i + 1);
            Lane lane = ((CrossSectionLink) getNetwork().getLink(linkId)).getLanes().get(laneNum);
            Speed speedLimit = Speed.ZERO;
            try
            {
                speedLimit = lane.getLowestSpeedLimit();
            }
            catch (NetworkException ex)
            {
                //
            }
            out.add(new Section<>(lane.getLength(), speedLimit, List.of(new LaneDataRoad(lane))));
        }
        return out;
    }

    /**
     * Extended data type for attention.
     */
    public static class DataTypeAttention extends ExtendedDataNumber<GtuDataRoad>
    {
        /** Channel predicate. */
        private final Predicate<Object> predicate;

        /**
         * Constructor.
         * @param id id
         * @param predicate channel predicate
         */
        DataTypeAttention(final String id, final Predicate<Object> predicate)
        {
            super(id, "attention " + id);
            this.predicate = predicate;
        }

        @Override
        public Float getValue(final GtuDataRoad gtu)
        {
            if (gtu.getGtu().getStrategicalPlanner() != null)
            {
                ChannelFuller fuller = (ChannelFuller) gtu.getGtu().getTacticalPlanner().getPerception().getMental();
                float result = 0.0f;
                for (Object channel : fuller.getChannels())
                {
                    if (this.predicate.test(channel))
                    {
                        result += (float) fuller.getAttention(channel);
                    }
                }
                return result;
            }
            return Float.NaN;
        }
    }

}
