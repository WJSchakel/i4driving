package org.opentrafficsim.i4driving.validation;

import java.awt.Color;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.rmi.RemoteException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djutils.cli.CliUtil;
import org.djutils.draw.point.OrientedPoint2d;
import org.djutils.draw.point.Point2d;
import org.djutils.event.Event;
import org.djutils.event.EventListener;
import org.djutils.exceptions.Try;
import org.opentrafficsim.animation.gtu.colorer.AccelerationGtuColorer;
import org.opentrafficsim.animation.gtu.colorer.IdGtuColorer;
import org.opentrafficsim.animation.gtu.colorer.SpeedGtuColorer;
import org.opentrafficsim.animation.gtu.colorer.SwitchableGtuColorer;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.definitions.Defaults;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.geometry.OtsGeometryException;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.gtu.RelativePosition;
import org.opentrafficsim.core.network.Network;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.i4driving.demo.AttentionAnimation;
import org.opentrafficsim.i4driving.demo.AttentionAnimation.ChannelAttention;
import org.opentrafficsim.i4driving.demo.TaskSaturationChannelColorer;
import org.opentrafficsim.i4driving.sim0mq.GtuSpawnerOd;
import org.opentrafficsim.i4driving.sim0mq.ParameterFactorySim0mq;
import org.opentrafficsim.i4driving.tactical.ScenarioTacticalPlanner;
import org.opentrafficsim.i4driving.tactical.ScenarioTacticalPlannerFactory;
import org.opentrafficsim.i4driving.tactical.ScenarioTacticalPlannerFactory.CarFollowing;
import org.opentrafficsim.i4driving.tactical.VisibilityLanePerception.Visibility;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTask;
import org.opentrafficsim.road.gtu.generator.characteristics.DefaultLaneBasedGtuCharacteristicsGeneratorOd;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.perception.structure.LaneStructure.Entry;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalRoutePlannerFactory;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.factory.xml.parser.XmlParser;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.conflict.Conflict;
import org.opentrafficsim.swing.graphs.SwingPlot;
import org.opentrafficsim.swing.gui.OtsSimulationApplication;
import org.opentrafficsim.swing.script.AbstractSimulationScript;

import de.siegmar.fastcsv.reader.CsvReader;
import de.siegmar.fastcsv.reader.CsvRow;
import nl.tudelft.simulation.dsol.animation.d2.Renderable2d;
import nl.tudelft.simulation.dsol.swing.gui.TablePanel;
import picocli.CommandLine.Mixin;
import picocli.CommandLine.Option;

/**
 * This simulation creates trajectory data that includes attention distribution.
 * @author wjschakel
 */
public class UrbanIntersection extends AbstractSimulationScript
{

    /** */
    private static final long serialVersionUID = 1L;

    /** GTU spawner. */
    private GtuSpawnerOd spawner;

    /** Tactical planner factory. */
    @Mixin
    private final ScenarioTacticalPlannerFactory tacticalFactory = new ScenarioTacticalPlannerFactory();

    @Option(names = {"--scenario"}, description = "Scenario", defaultValue = "fp15_Bike14")
    private String scenario;

    @Option(names = {"--dataStep"}, description = "Step size in data", defaultValue = "0.05s")
    private Duration dataStep;

    /** Rendered GTUs. */
    protected Map<LaneBasedGtu, Renderable2d<ChannelAttention>> animatedGTUs =
            Collections.synchronizedMap(new LinkedHashMap<>());

    /** Parameter factory. */
    private ParameterFactorySim0mq parameterFactory;

    /**
     * Constructor.
     */
    public UrbanIntersection()
    {
        super("Urban", "Attention validation at urban intersection.");
    }

    /**
     * Main program.
     * @param args command line arguments. See AbstractSimulationScript for available arguments.
     * @throws Exception when an exception occurs.
     */
    public static void main(final String[] args) throws Exception
    {
        UrbanIntersection demo = new UrbanIntersection();
        CliUtil.changeOptionDefault(demo, "simulationTime", "60s");
        CliUtil.execute(demo, args);
        demo.tacticalFactory.setConflictsInfra(true);
        demo.setGtuColorer(
                new SwitchableGtuColorer(3, new IdGtuColorer(), new SpeedGtuColorer(new Speed(150, SpeedUnit.KM_PER_HOUR)),
                        new AccelerationGtuColorer(Acceleration.instantiateSI(-6.0), Acceleration.instantiateSI(2)),
                        new TaskSaturationChannelColorer()));
        demo.start();
    }

    @Override
    protected RoadNetwork setupSimulation(final OtsSimulatorInterface sim) throws Exception
    {
        RoadNetwork network = new RoadNetwork("Urban", sim);
        InputStream stream = getClass().getResourceAsStream("/val/VTI.xml");
        XmlParser parser = new XmlParser(network).setStream(stream).setParseConflict(true);
        parser.build();

        Visibility visibility = new Visibility();
        visibility.addAnchor(network, new Point2d(13.0, -7.5));
        visibility.addAnchor(network, new Point2d(7.5, -13.0));
        visibility.addAnchor(network, new Point2d(-13.0, -7.5));
        visibility.addAnchor(network, new Point2d(-7.5, -13.0));
        this.tacticalFactory.setVisibility(visibility);

        GtuType.registerTemplateSupplier(DefaultsNl.CAR, Defaults.NL);
        this.tacticalFactory.setStream(sim.getModel().getStream("generation"));
        this.tacticalFactory.setCarFollowing(CarFollowing.IDM_PLUS);
        this.tacticalFactory.setLocalDistraction(false);
        this.tacticalFactory.setFreeAccelerationTask(false);
        this.tacticalFactory.setLaneChangingTask(false);
        this.tacticalFactory.setTrafficLightsTask(false);
        this.tacticalFactory.setSignalTask(false);
        this.tacticalFactory.setCooperationTask(false);
        this.tacticalFactory.setSocioLaneChange(false);
        this.tacticalFactory.setActiveMode(false);
        this.tacticalFactory.setUpdateTimeAdaptation(false);
        this.tacticalFactory.setNumberOfLeaders(1);

        // Parameters
        this.parameterFactory = new ParameterFactorySim0mq();
        this.parameterFactory.addParameter(Fuller.TC, 1.0);
        DefaultLaneBasedGtuCharacteristicsGeneratorOd gtuCharacteristicsGenerator =
                new DefaultLaneBasedGtuCharacteristicsGeneratorOd.Factory(
                        new LaneBasedStrategicalRoutePlannerFactory(this.tacticalFactory, this.parameterFactory)).create();

        this.spawner = new GtuSpawnerOd(network, gtuCharacteristicsGenerator);

        Map<String, double[]> vehicleData = readColumns(getClass().getResourceAsStream("/val/" + this.scenario + "_sim.csv"));
        for (String vehicle : new String[] {"ego", "car_conflict1", "car_conflict2", "car_conflict3", "car_merge1",
                "car_merge2", "car_merge3"})
        {
            if (vehicleData.containsKey(vehicle + "_x"))
            {
                double[] x = vehicleData.get(vehicle + "_x");
                double[] y = vehicleData.get(vehicle + "_y");
                double[] v = vehicleData.get(vehicle + "_v");
                sim.scheduleEventAbs(this.dataStep, () -> spawnByData(vehicle, x, y, v));
            }
        }

        // sim.scheduleEventAbs(Duration.ONE,
        // () -> spawnFixedSpeed("Ego", "S1", "W4", new Speed(50.0, SpeedUnit.KM_PER_HOUR), Length.instantiateSI(70.0)));
        // sim.scheduleEventAbs(Duration.ONE,
        // () -> spawn("E1", "E1", "W4", new Speed(50.0, SpeedUnit.KM_PER_HOUR), Length.instantiateSI(90.0)));
        // sim.scheduleEventAbs(Duration.ONE,
        // () -> spawn("E2", "E1", "W4", new Speed(50.0, SpeedUnit.KM_PER_HOUR), Length.instantiateSI(60.0)));
        // sim.scheduleEventAbs(Duration.ONE,
        // () -> spawn("E3", "E1", "W4", new Speed(50.0, SpeedUnit.KM_PER_HOUR), Length.instantiateSI(30.0)));
        // sim.scheduleEventAbs(Duration.ONE,
        // () -> spawn("W1", "W1", "E4", new Speed(50.0, SpeedUnit.KM_PER_HOUR), Length.instantiateSI(90.0)));
        // sim.scheduleEventAbs(Duration.ONE,
        // () -> spawn("W2", "W1", "E4", new Speed(50.0, SpeedUnit.KM_PER_HOUR), Length.instantiateSI(60.0)));
        // sim.scheduleEventAbs(Duration.ONE,
        // () -> spawn("W3", "W1", "E4", new Speed(50.0, SpeedUnit.KM_PER_HOUR), Length.instantiateSI(30.0)));

        // Attention animation
        EventListener ev = new EventListener()
        {
            /** */
            private static final long serialVersionUID = 20251007L;

            @Override
            public void notify(final Event event) throws RemoteException
            {
                LaneBasedGtu gtu = (LaneBasedGtu) network.getGTU((String) event.getContent());
                if (event.getType().equals(Network.GTU_ADD_EVENT))
                {
                    // schedule the addition of the GTU to prevent it from not having an operational plan
                    gtu.getSimulator().scheduleEventNow(() -> UrbanIntersection.this.animateGTU(gtu));
                    // ScenarioConflict.this.animatedGTUs.put(gtu, new AttentionAnimation(gtu, gtu.getSimulator()));
                }
                else if (event.getType().equals(Network.GTU_REMOVE_EVENT))
                {
                    if (UrbanIntersection.this.animatedGTUs.containsKey(gtu))
                    {
                        UrbanIntersection.this.animatedGTUs.get(gtu).destroy(gtu.getSimulator());
                        UrbanIntersection.this.animatedGTUs.remove(gtu);
                    }
                }
            }
        };
        network.addListener(ev, Network.GTU_ADD_EVENT);
        network.addListener(ev, Network.GTU_REMOVE_EVENT);

        return network;
    }

    /**
     * Spawn GTU with fixed speed.
     * @param id GTU id
     * @param nodeFrom from node
     * @param nodeTo to node
     * @param speed initial speed
     * @param position initial position
     */
    private void spawnFixedSpeed(final String id, final String nodeFrom, final String nodeTo, final Speed speed,
            final Length position)
    {
        try
        {
            Route route = getRoute(nodeFrom, nodeTo);
            Lane lane = ((CrossSectionLink) route.getNode(0).getLinks().iterator().next()).getLanes().get(0);
            Point2d location = lane.getCenterLine().getLocation(position);
            this.spawner.spawnGtu(id, DefaultsNl.CAR, Length.instantiateSI(4.0), Length.instantiateSI(1.75),
                    Length.instantiateSI(3.0), route, speed, location);
            if (!"Ego".equals(id))
            {
                ((ScenarioTacticalPlanner) getNetwork().getGTU(id).getTacticalPlanner()).setDesiredSpeed(speed);
            }
        }
        catch (NetworkException | IllegalArgumentException | GtuException | OtsGeometryException ex)
        {
            throw new RuntimeException(ex);
        }
    }

    /**
     * Spawn vehicle based on data.
     * @param id id
     * @param x x array
     * @param y y array
     * @param v speed array
     */
    private void spawnByData(final String id, final double[] x, final double[] y, final double[] v)
    {
        try
        {
            Route route;
            if ("ego".equals(id.toLowerCase()))
            {
                route = x[x.length - 1] < 0.0 ? getRoute("S1", "W4") : getRoute("S1", "E4");
            }
            else
            {
                route = x[0] < 0.0 ? getRoute("W1", "E4") : getRoute("E1", "W4");
            }
            if ("ego".equals(id))
            {
                // fSpeed in accordance with first 3s of data
                int steps = (int) (3 * (1 / this.dataStep.si));
                double speed = Arrays.stream(v).limit(steps).average().getAsDouble();
                double fSpeed = speed / (50.0 / 3.6);
                this.parameterFactory.setParameterValue(ParameterTypes.FSPEED, fSpeed);
            }

            this.spawner.spawnGtu(id, DefaultsNl.CAR, Length.instantiateSI(4.0), Length.instantiateSI(1.75),
                    Length.instantiateSI(3.0), route, Speed.instantiateSI(v[0]), new Point2d(x[0], y[0]));
            LaneBasedGtu gtu = (LaneBasedGtu) getNetwork().getGTU(id);
            if ("ego".equals(id))
            {
                this.parameterFactory.clearParameterValue(ParameterTypes.FSPEED);
            }
            else
            {
                getSimulator().scheduleEventRel(this.dataStep, () -> deadReckoning(gtu, x, y, v, 1));
            }
        }
        catch (NetworkException | IllegalArgumentException | GtuException | OtsGeometryException ex)
        {
            throw new RuntimeException(ex);
        }
    }

    /**
     * Step-wise dead-reackoning on GTU.
     * @param gtu GTU
     * @param x x coordinates
     * @param y y coordinates
     * @param v speed
     * @param index index in data to use
     */
    private void deadReckoning(final LaneBasedGtu gtu, final double[] x, final double[] y, final double[] v, final int index)
    {
        if (gtu.isDestroyed())
        {
            return;
        }
        int dirIndex = index < x.length - 1 ? index : index - 1;
        double dir = new Point2d(x[dirIndex], y[dirIndex]).directionTo(new Point2d(x[dirIndex + 1], y[dirIndex + 1]));
        OrientedPoint2d point = new OrientedPoint2d(x[index], y[index], dir);

        ((ScenarioTacticalPlanner) gtu.getTacticalPlanner()).deadReckoning(point, Speed.instantiateSI(v[index]),
                Acceleration.ZERO);
        if (index < x.length - 1 && Math.abs(x[index]) < 5.0 || Math.abs(y[index]) < 5.0)
        {
            getSimulator().scheduleEventRel(this.dataStep, () -> deadReckoning(gtu, x, y, v, index + 1));
        }
        else
        {
            gtu.destroy();
        }
    }

    /**
     * Get route between nodes.
     * @param from from node id
     * @param to to node id
     * @return route between nodes
     * @throws NetworkException if no route exists between the nodes for NL.CAR
     */
    private Route getRoute(String from, String to) throws NetworkException
    {
        return getNetwork().getShortestRouteBetween(DefaultsNl.CAR, getNetwork().getNode(from), getNetwork().getNode(to));
    }

    /**
     * Draw the attention.
     * @param gtu the GTU to draw the attention of
     */
    @SuppressWarnings("unused") // scheduled
    private void animateGTU(final LaneBasedGtu gtu)
    {
        Renderable2d<ChannelAttention> gtuAnimation = new AttentionAnimation(new ChannelAttention(gtu), gtu.getSimulator());
        this.animatedGTUs.put(gtu, gtuAnimation);
    }

    @Override
    protected void addTabs(final OtsSimulatorInterface sim, final OtsSimulationApplication<?> animation)
    {
        TablePanel plotPanel = new TablePanel(1, 1);
        animation.getAnimationPanel().getTabbedPane().addTab("plots", plotPanel);

        // -sum(AR) => Anticipation Reliance [dark purple]
        // 1.0 - sum(ATT) => Idle [brown]
        // ChannelTask.FRONT => Front, Conflict from N [brown]
        // ChannelTask.RIGHT => Right, conflict from E [dark yellow]
        // ChannelTask.LEFT => Left, conflict from W [blue]
        // ChannelTask.REAR => Rear [dark blue]

        List<Object> defaultChannels = List.of(ChannelTask.FRONT, ChannelTask.RIGHT, ChannelTask.LEFT, ChannelTask.REAR);
        List<Color> colors =
                List.of(new Color(155, 67, 66), new Color(251, 203, 107), new Color(116, 164, 252), new Color(100, 77, 108));

        AttentionContext attentionContext = new AttentionContext()
        {
            @Override
            public RoadNetwork getNetwork()
            {
                return UrbanIntersection.this.getNetwork();
            }

            @Override
            public String getGtuId()
            {
                return "ego";
            }

            @Override
            public int getNumberOfGroups()
            {
                return 4;
            }

            @Override
            public int getGroupIndex(final Object channel)
            {
                if (defaultChannels.contains(channel))
                {
                    return defaultChannels.indexOf(channel);
                }
                Conflict conflict = (Conflict) channel;

                // Point2d point = conflict.getOtherConflict().getLane().getLink().getStartNode().getLocation();
                // LaneBasedGtu gtu = (LaneBasedGtu) conflict.getLane().getNetwork().getGTU(getGtuId());
                // Double angle = Math.toDegrees(gtu.getLocation().directionTo(point) - gtu.getLocation().dirZ);
                // if (angle < -10.0)
                // {
                // return 1;
                // }
                // else if (angle > 10.0)
                // {
                // return 2;
                // }
                // return 0;

                String conflictingNode = conflict.getOtherConflict().getLane().getLink().getStartNode().getId();

                // LaneBasedGtu gtu = (LaneBasedGtu) conflict.getLane().getNetwork().getGTU(getGtuId());
                // if (!conflictingNode.equals("N2") && gtu.getLocation().y < -10.0)
                // {
                // double angle;
                // if (conflictingNode.equals("E2"))
                // {
                // angle = Math.atan(8.25 / (-10.0 - gtu.getLocation().y));
                // }
                // else
                // {
                // angle = Math.atan(11.25 / (-10.0 - gtu.getLocation().y));
                // }
                // if (Math.toDegrees(angle) < 20.0)
                // {
                // // conflict visibility is within 10 degree cone; attention is mapped to front
                // return 0;
                // }
                // }

                switch (conflictingNode)
                {
                    case "N2":
                        return 0;
                    case "E2":
                        return 1;
                    case "W2":
                        return 2;
                    default:
                        throw new RuntimeException("Unknown channel " + channel);
                }
            }

            @Override
            public int getGroupIndexByLabel(final String groupLabel)
            {
                return defaultChannels.indexOf(groupLabel);
            }

            @Override
            public String getGroupLabel(final int groupIndex)
            {
                return (String) defaultChannels.get(groupIndex);
            }

            @Override
            public Color getGroupColor(final int groupIndex)
            {
                return colors.get(groupIndex);
            }

            @Override
            public Color getIdleColor()
            {
                return colors.get(0).darker();
            }

            @Override
            public Color getAnticipationRelianceColor()
            {
                return colors.get(3).brighter();
            }

            @Override
            public Length getZeroOdometer(final LaneBasedGtu gtu)
            {
                Iterable<Entry<Conflict>> conflicts = Try.assign(
                        () -> gtu.getTacticalPlanner().getPerception().getLaneStructure()
                                .getDownstreamObjects(RelativeLane.CURRENT, Conflict.class, RelativePosition.CENTER, true),
                        RuntimeException.class, "Unable to find conflicts from lane structure.");
                for (Entry<Conflict> conflict : conflicts)
                {
                    if (conflict.object().getConflictType().isCrossing() || conflict.object().getConflictType().isMerge())
                    {
                        return conflict.distance().plus(gtu.getOdometer());
                    }
                }
                return Length.ZERO;
            }

            @Override
            public Length getMaxDistance(final LaneBasedGtu gtu)
            {
                return Length.instantiateSI(15.0).plus(gtu.getCenter().dx());
            }

        };
        AttentionDistributionPlot plot = new AttentionDistributionPlot(attentionContext);
        plot.setLowerDomainBound(0.0);
        plot.setUpperRangeBound(1.0);
        SwingPlot swingPlot = new SwingPlot(plot);
        plotPanel.setCell(swingPlot.getContentPane(), 0, 0);
    }

    /**
     * Reads a CSV file input stream with a header row and numeric data, returning a {@link Map} of column name
     * {@code -> double[]}.<br>
     * <br>
     * This method was created with the help of ChatGPT (which needed 5 corrections).
     * @param csvStream input stream of CSV file
     */
    private static Map<String, double[]> readColumns(InputStream csvStream) throws IOException
    {

        CsvReader reader = CsvReader.builder().build(new InputStreamReader(csvStream));

        try (Stream<CsvRow> stream = reader.stream())
        {

            Iterator<CsvRow> iterator = stream.iterator();

            if (!iterator.hasNext())
            {
                throw new IllegalArgumentException("CSV file is empty");
            }

            // --- Header ---
            CsvRow headerRow = iterator.next();
            List<String> headers = headerRow.getFields();
            int columnCount = headers.size();

            // --- Temporary storage (preserve order) ---
            Map<String, List<Double>> tempColumns = new LinkedHashMap<>();
            for (String header : headers)
            {
                tempColumns.put(header, new ArrayList<>());
            }

            // --- Data rows ---
            int rowIndex = 1;
            while (iterator.hasNext())
            {
                CsvRow row = iterator.next();

                if (row.getFieldCount() != columnCount)
                {
                    throw new IllegalArgumentException("Row " + rowIndex + " has incorrect column count");
                }

                for (int c = 0; c < columnCount; c++)
                {
                    double value = Double.parseDouble(row.getField(c));
                    tempColumns.get(headers.get(c)).add(value);
                }

                rowIndex++;
            }

            // --- Convert to double[] ---
            Map<String, double[]> result = new LinkedHashMap<>();
            for (Map.Entry<String, List<Double>> entry : tempColumns.entrySet())
            {
                List<Double> list = entry.getValue();
                double[] array = new double[list.size()];
                for (int i = 0; i < list.size(); i++)
                {
                    array[i] = list.get(i);
                }
                result.put(entry.getKey(), array);
            }

            return result;
        }
    }

}
