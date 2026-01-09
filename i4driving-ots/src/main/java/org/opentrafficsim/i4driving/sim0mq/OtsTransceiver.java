package org.opentrafficsim.i4driving.sim0mq;

import java.awt.Dimension;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.rmi.RemoteException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Function;

import javax.naming.NamingException;
import javax.xml.bind.JAXBException;
import javax.xml.parsers.ParserConfigurationException;

import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Direction;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.Time;
import org.djunits.value.vfloat.vector.FloatAccelerationVector;
import org.djunits.value.vfloat.vector.FloatDurationVector;
import org.djunits.value.vfloat.vector.FloatLengthVector;
import org.djutils.cli.CliUtil;
import org.djutils.draw.point.OrientedPoint2d;
import org.djutils.event.Event;
import org.djutils.event.EventListener;
import org.djutils.event.EventType;
import org.djutils.immutablecollections.ImmutableList;
import org.djutils.logger.CategoryLogger;
import org.djutils.metadata.MetaData;
import org.djutils.serialization.EndianUtil;
import org.djutils.serialization.SerializationException;
import org.djutils.serialization.TypedMessage;
import org.djutils.serialization.serializers.Serializer;
import org.opentrafficsim.animation.colorer.SynchronizationColorer;
import org.opentrafficsim.animation.gtu.colorer.AccelerationGtuColorer;
import org.opentrafficsim.animation.gtu.colorer.GtuColorer;
import org.opentrafficsim.animation.gtu.colorer.IdGtuColorer;
import org.opentrafficsim.animation.gtu.colorer.SpeedGtuColorer;
import org.opentrafficsim.animation.gtu.colorer.SwitchableGtuColorer;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterType;
import org.opentrafficsim.core.definitions.Defaults;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.dsol.AbstractOtsModel;
import org.opentrafficsim.core.dsol.OtsAnimator;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.geometry.OtsGeometryException;
import org.opentrafficsim.core.geometry.OtsLine2d;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlan;
import org.opentrafficsim.core.gtu.plan.operational.Segment;
import org.opentrafficsim.core.idgenerator.IdGenerator;
import org.opentrafficsim.core.network.Network;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.core.perception.HistoryManagerDevs;
import org.opentrafficsim.draw.OtsDrawingException;
import org.opentrafficsim.i4driving.messages.Commands;
import org.opentrafficsim.i4driving.messages.DefaultGson;
import org.opentrafficsim.i4driving.object.ActiveModeCrossing;
import org.opentrafficsim.i4driving.tactical.CommandsHandler;
import org.opentrafficsim.i4driving.tactical.NetworkUtil;
import org.opentrafficsim.i4driving.tactical.ScenarioTacticalPlanner;
import org.opentrafficsim.i4driving.tactical.ScenarioTacticalPlannerFactory;
import org.opentrafficsim.road.definitions.DefaultsRoadNl;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuCharacteristicsGeneratorOd;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.lane.LanePosition;
import org.opentrafficsim.road.od.OdApplier;
import org.opentrafficsim.road.od.OdMatrix;
import org.opentrafficsim.road.od.OdOptions;
import org.opentrafficsim.swing.gui.OtsAnimationPanel;
import org.opentrafficsim.swing.gui.OtsSimulationApplication;
import org.pmw.tinylog.Level;
import org.sim0mq.Sim0MQException;
import org.sim0mq.message.Sim0MQMessage;
import org.xml.sax.SAXException;
import org.zeromq.SocketType;
import org.zeromq.ZContext;
import org.zeromq.ZMQ;

import com.google.gson.Gson;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.dsol.simulators.SimulatorInterface;
import nl.tudelft.simulation.language.DsolException;
import picocli.CommandLine.Command;
import picocli.CommandLine.Mixin;
import picocli.CommandLine.Option;

/**
 * OTS co-simulation server.
 * @author wjschakel
 */
@Command(description = "OTS Transceiver for co-simulation", name = "OTS", mixinStandardHelpOptions = true,
        showDefaultValues = true, version = "20250619")
public class OtsTransceiver
{

    /** Federation id to receive/sent messages. */
    @Option(names = "--federationId", description = "Federation id to receive/sent messages", defaultValue = "Ots_ExternalSim")
    private String federation;

    /** Ots id to receive/sent messages. */
    @Option(names = "--otsId", description = "Ots id to receive/sent messages", defaultValue = "Ots")
    private String ots;

    /** Client id to receive/sent messages. */
    @Option(names = "--clientId", description = "Client id to receive/sent messages", defaultValue = "ExternalSim")
    private String client;

    /** Endianness. */
    @Option(names = "--bigEndian", description = "Big-endianness", defaultValue = "false", negatable = true)
    private Boolean bigEndian;

    /** Port number. */
    @Option(names = "--port", description = "Port number", defaultValue = "5556")
    private int port;

    /** Show GUI. */
    @Option(names = "--no-gui", description = "Whether to show GUI", defaultValue = "false", negatable = true) // false=default
    private boolean showGui;

    /** ID prefix of generated GTUs. */
    @Option(names = "--idPrefix", description = "Prefix to ID of generated traffic.", defaultValue = "OTS_")
    private String idPrefix;

    /** Whether to use the road name to identify origins and destinations. */
    @Option(names = "--useRoadName", description = "Whether to use the road name to identify origins and destinations.",
            defaultValue = "false")
    private boolean useRoadName;

    /** Mixed in model arguments. */
    @Mixin
    private ScenarioTacticalPlannerFactory tacticalFactory = new ScenarioTacticalPlannerFactory();

    /**
     * Constructor.
     * @param args command line arguments.
     * @throws Exception on any exception during simulation.
     */
    protected OtsTransceiver(final String... args) throws Exception
    {
        CliUtil.execute(this, args);
    }

    /**
     * Main method.
     * @param args command line arguments
     * @throws Exception exception
     */
    public static void main(final String[] args) throws Exception
    {
        new OtsTransceiver(args).start();
    }

    /**
     * Starts worker thread.
     */
    private void start()
    {
        new Worker().start();
    }

    /**
     * Worker thread to listen to messages and respond.
     * <p>
     * Copyright (c) 2024-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.
     * <br>
     * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
     * </p>
     * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
     */
    protected class Worker extends Thread implements EventListener
    {

        /** */
        private static final long serialVersionUID = 20241210L;

        /** */
        private ZContext context;

        /** the socket. */
        private ZMQ.Socket responder;

        /** Messages to be sent. */
        private ConcurrentLinkedQueue<QueuedMessage> queue = new ConcurrentLinkedQueue<>();

        /** Next message id. */
        private int messageId = 0;

        /** Last OD matrix sent. */
        private OdMatrixJson lastOdJson;

        /** Last routes sent. */
        private RoutesJson lastRoutesJson;

        /** Last network message. */
        private Sim0MQMessage lastNetworkMessage;

        /** List of vehicle message payloads before simulation start. */
        private List<Object[]> preStartVehiclePayloads = new ArrayList<>();

        /** GTU characteristics generator. */
        private LaneBasedGtuCharacteristicsGeneratorOd characteristicsGeneratorOd;

        /** Parameter factory. */
        private ParameterFactorySim0mq parameterFactory;

        /** GTU spawner. */
        private GtuSpawnerOd gtuSpawner;

        /** Simulator. */
        private OtsAnimator simulator;

        /** Network. */
        private RoadNetwork network;

        /** Application. */
        private OtsSimulationApplication<AbstractOtsModel> app;

        /** GTU that is being externally generated, for which no VEHICLE message should be sent. */
        private String externallyGeneratedGtuId;

        /** Ids of GTUs for which plan messages are sent. */
        private Set<String> planGtuIds = new LinkedHashSet<>();

        /** Ids of GTUs that are externally controlled. */
        private Set<String> externalGtuIds = new LinkedHashSet<>();

        /** Ids of active mode objects, and to which crossing they pertain. */
        private Map<String, ActiveModeCrossing> activeIds = new LinkedHashMap<>();

        /** Command handlers. */
        private Map<String, CommandsHandler> commandHandlers = new LinkedHashMap<>();

        /** Ids of GTUs that are externally deleted, i.e. do not sent back a delete message. */
        private Set<String> deleteGtuIds = new LinkedHashSet<>();

        /** GSON builder to parser JSON strings. */
        private Gson gson = DefaultGson.GSON;

        /** Run until time. */
        private Duration runUntil;

        /** Message id of the last progress message. */
        private int progressMessageId;

        /** {@inheritDoc} */
        @Override
        public void run()
        {
            this.context = new ZContext(1);
            this.responder = this.context.createSocket(SocketType.PAIR);
            this.responder.bind("tcp://*:" + port);
            CategoryLogger.setAllLogLevel(Level.DEBUG);
            CategoryLogger.setAllLogMessageFormat("[{date: YYYY-MM-dd HH:mm:ss.SSS}] {level}: {message}");
            CategoryLogger.always().debug("Ots is running");

            try
            {
                // Note on synchronicity and possible dead-locks:
                // OTS is single-threaded. All changes during the simulation should be scheduled in the simulator. All messages
                // sent back from a notification from simulation, should be queued for the Worker thread in the queue.
                while (!Thread.currentThread().isInterrupted())
                {
                    // Wait for next request from the client
                    byte[] request = null;
                    request = this.responder.recv(ZMQ.DONTWAIT);
                    while (request == null)
                    {
                        QueuedMessage send = this.queue.poll();
                        while (send != null)
                        {
                            this.responder.send(send.message(), ZMQ.DONTWAIT);
                            if (send.log() != null)
                            {
                                CategoryLogger.always().debug("[q] " + send.log());
                            }
                            send = this.queue.poll();
                        }

                        try
                        {
                            // In order to allow resources to go to other processes, we sleep before checking again
                            Thread.sleep(3);
                        }
                        catch (InterruptedException e)
                        {
                        }
                        request = this.responder.recv(ZMQ.DONTWAIT);
                    }
                    // Sim0MQMessage message = Sim0MQMessage.decode(request);
                    Object[] array = TypedMessage.decode(request, OBJECT_DECODERS,
                            request[11] == 1 ? EndianUtil.BIG_ENDIAN : EndianUtil.LITTLE_ENDIAN);
                    Sim0MQMessage message = new Sim0MQMessage(array, array.length - 8, array[5]);
                    if ("EXTERNAL".equals(message.getMessageTypeId()))
                    {
                        Object[] payload = message.createObjectArray();
                        int index = 8;
                        String id = (String) payload[index++];
                        Length x = (Length) payload[index++];
                        Length y = (Length) payload[index++];
                        Direction direction = (Direction) payload[index++];
                        Speed speed = (Speed) payload[index++];
                        Acceleration acceleration = (Acceleration) payload[index++];
                        OrientedPoint2d loc = new OrientedPoint2d(x.si, y.si, direction.si);
                        if (this.activeIds.containsKey(id))
                        {
                            this.simulator.scheduleEventNow(this, "updateActiveModeObject", new Object[] {id, loc, speed});
                        }
                        else
                        {
                            this.simulator.scheduleEventNow(this, "scheduledDeadReckoning",
                                    new Object[] {id, loc, speed, acceleration});
                        }
                        CategoryLogger.always().debug("Ots received EXTERNAL message for GTU " + id);
                    }
                    else if ("VEHICLE".equals(message.getMessageTypeId()))
                    {
                        Object[] payload = message.createObjectArray();
                        int index = 8;
                        String id = (String) payload[index++];
                        CategoryLogger.always().debug("Ots received VEHICLE message for GTU " + id);
                        generateVehicle(payload, true);
                    }
                    else if ("MODE".equals(message.getMessageTypeId()))
                    {
                        Object[] payload = message.createObjectArray();
                        String id = (String) payload[8];
                        CategoryLogger.always().debug("Ots received MODE message for GTU " + id);
                        String mode = (String) payload[9];
                        this.simulator.scheduleEventNow(this, "scheduledChangeControlMode", new Object[] {id, mode});
                    }
                    else if ("COMMAND".equals(message.getMessageTypeId()))
                    {
                        Object[] payload = message.createObjectArray();
                        String id = (String) payload[8];
                        CategoryLogger.always().debug("Ots received COMMAND message for GTU " + id);
                        String json = (String) payload[9];
                        this.simulator.scheduleEventNow(this, "scheduledPerformCommand", new Object[] {id, json});
                    }
                    else if ("DELETE".equals(message.getMessageTypeId()))
                    {
                        Object[] payload = message.createObjectArray();
                        String id = (String) payload[8];
                        CategoryLogger.always().debug("Ots received DELETE message for GTU " + id);
                        this.deleteGtuIds.add(id);
                        this.simulator.scheduleEventNow(this, "scheduledDelete", new Object[] {id});
                    }
                    else if ("ROUTES".equals(message.getMessageTypeId()))
                    {
                        CategoryLogger.always().debug("Ots received ROUTES message");
                        Object[] payload = message.createObjectArray();
                        this.lastRoutesJson = DefaultGson.GSON.fromJson((String) payload[8], RoutesJson.class);
                        sentReadyMessage((int) message.createObjectArray()[6], false);
                    }
                    else if ("ODMATRIX".equals(message.getMessageTypeId()))
                    {
                        CategoryLogger.always().debug("Ots received ODMATRIX message");
                        Object[] payload = message.createObjectArray();
                        this.lastOdJson = DefaultGson.GSON.fromJson((String) payload[8], OdMatrixJson.class);
                        sentReadyMessage((int) message.createObjectArray()[6], false);
                    }
                    else if ("NETWORK".equals(message.getMessageTypeId()))
                    {
                        CategoryLogger.always().debug("Ots received NETWORK message");
                        this.lastNetworkMessage = message;
                        setupSimulation();
                        sentReadyMessage((int) message.createObjectArray()[6], false);
                    }
                    else if ("START".equals(message.getMessageTypeId()))
                    {
                        CategoryLogger.always().debug("Ots received START message");
                        this.simulator.setSpeedFactor(1.0);
                        if (this.simulator != null && !this.simulator.isStartingOrRunning())
                        {
                            this.simulator.start();
                        }
                    }
                    else if ("STOP".equals(message.getMessageTypeId()))
                    {
                        CategoryLogger.always().debug("Ots received STOP message");
                        stopSimulation();
                        clearSimulationSetupData();
                    }
                    else if ("RESET".equals(message.getMessageTypeId()))
                    {
                        CategoryLogger.always().debug("Ots received RESET message");
                        setupSimulation();
                        sentReadyMessage((int) message.createObjectArray()[6], false);
                    }
                    else if ("TERMINATE".equals(message.getMessageTypeId()))
                    {
                        CategoryLogger.always().debug("Ots received TERMINATE message");
                        stopSimulation();
                        clearSimulationSetupData();
                        break;
                    }
                    else if ("PROGRESS".equals(message.getMessageTypeId()))
                    {
                        Object[] payload = message.createObjectArray();
                        Duration until = (Duration) payload[8];
                        CategoryLogger.always().debug("Ots received PROGRESS message until {}", until);
                        this.simulator.setSpeedFactor(1000.0);
                        while (this.simulator.isStartingOrRunning())
                        {
                            try
                            {
                                // Simulator is still stopping from previous step
                                System.out.println("Waiting for next PROGRESS");
                                Thread.sleep(3);
                            }
                            catch (InterruptedException e)
                            {
                            }
                        }
                        this.simulator.scheduleEventAbs(until, () -> fireProgressedEvent());
                        this.simulator.runUpToAndIncluding(until);
                        this.progressMessageId = (int) payload[6];
                        this.runUntil = until;
                    }
                    else
                    {
                        System.err.println("Cannot process a " + message.getMessageTypeId() + " message.");
                    }
                }
            }
            catch (Sim0MQException | SerializationException | NumberFormatException | GtuException | OtsGeometryException
                    | NetworkException | RemoteException | DsolException | OtsDrawingException | SimRuntimeException
                    | NamingException | ParameterException | JAXBException | SAXException | ParserConfigurationException
                    | IllegalAccessException | InvocationTargetException e)
            {
                e.printStackTrace();
            }
            this.responder.close();
            this.context.destroy();
            this.context.close();
            CategoryLogger.always().debug("Ots terminated");
            System.exit(0);
        }

        /**
         * Generates vehicle.
         * @param payload message payload
         * @param addToPreStartList whether to add the message to the list of pre-start vehicle messages if not started yet
         * @throws GtuException exception
         * @throws OtsGeometryException exception
         * @throws NetworkException exception
         * @throws RemoteException exception
         * @throws InvocationTargetException if a parameter cannot be set
         * @throws IllegalAccessException if a parameter cannot be set
         */
        private void generateVehicle(final Object[] payload, final boolean addToPreStartList) throws GtuException,
                OtsGeometryException, NetworkException, RemoteException, IllegalAccessException, InvocationTargetException
        {
            boolean running = this.simulator != null && this.simulator.getSimulatorTime().gt0();
            int index = 8;
            String id = (String) payload[index++];
            String mode = (String) payload[index++];
            Length initX = (Length) payload[index++];
            Length initY = (Length) payload[index++];
            Direction initDirection = (Direction) payload[index++];
            OrientedPoint2d position = new OrientedPoint2d(initX.si, initY.si, initDirection.si);
            Speed initSpeed = (Speed) payload[index++];
            if (mode.toLowerCase().equals("active"))
            {
                if (running)
                {
                    this.simulator.scheduleEventNow(this, "addActiveModeObject", new Object[] {id, position, initSpeed});
                }
                else
                {
                    addActiveModeObject(id, position, initSpeed);
                }
                return;
            }
            String vehicleType = ((String) payload[index++]).toUpperCase();
            GtuType gtuType =
                    Defaults.getByName(GtuType.class, vehicleType.startsWith("NL.") ? vehicleType : "NL." + vehicleType);
            Length vehicleLength = (Length) payload[index++];
            Length vehicleWidth = (Length) payload[index++];
            Length refToNose = (Length) payload[index++];
            int numParams = (int) payload[index++];
            Map<String, Object> parameterMap = new LinkedHashMap<>();
            for (int i = 0; i < numParams; i++)
            {
                parameterMap.put((String) payload[index++], payload[index++]);
            }
            Route route = this.network.getRoute((String) payload[index++]);

            this.externallyGeneratedGtuId = id;
            if (running)
            {
                this.simulator.scheduleEventNow(this, "spawnGtu", new Object[] {id, gtuType, vehicleLength, vehicleWidth,
                        refToNose, route, initSpeed, position, mode, parameterMap});
            }
            else
            {
                spawnGtu(id, gtuType, vehicleLength, vehicleWidth, refToNose, route, initSpeed, position, mode, parameterMap);
                sentReadyMessage((int) payload[6], false);
                if (addToPreStartList)
                {
                    this.preStartVehiclePayloads.add(payload);
                }
            }
        }

        /**
         * Spawn GTU scheduled.
         * @param id id
         * @param gtuType GTU type
         * @param vehicleLength length
         * @param vehicleWidth width
         * @param refToNose distance from reference point to front
         * @param route route
         * @param initSpeed speed
         * @param position position
         * @param mode mode
         * @param parameterMap map of parameters
         * @param <T> helper casting type
         * @throws GtuException when initial GTU values are not correct
         * @throws OtsGeometryException when the initial path is wrong
         * @throws NetworkException when the GTU cannot be placed on the given position
         * @throws InvocationTargetException if a parameter cannot be set
         * @throws IllegalAccessException if a parameter cannot be set
         */
        @SuppressWarnings("checkstyle:parameternumber")
        private <T extends Enum<T>> void spawnGtu(final String id, final GtuType gtuType, final Length vehicleLength,
                final Length vehicleWidth, final Length refToNose, final Route route, final Speed initSpeed,
                final OrientedPoint2d position, final String mode, final Map<String, Object> parameterMap)
                throws GtuException, OtsGeometryException, NetworkException, IllegalAccessException, InvocationTargetException
        {
            Set<ParameterType<?>> setParameters = new LinkedHashSet<>();
            for (Entry<String, Object> parameterEntry : parameterMap.entrySet())
            {
                String parameter = parameterEntry.getKey();
                Object value = parameterEntry.getValue();
                boolean singleShot = false;
                if (parameter.startsWith("--"))
                {
                    if (!singleShot)
                    {
                        OtsTransceiver.this.tacticalFactory.setSingleShotMode();
                        singleShot = true;
                    }
                    String methodName = "set" + parameter.substring(2).toLowerCase();
                    for (Method method : ScenarioTacticalPlannerFactory.class.getMethods())
                    {
                        if (method.getName().toLowerCase().equals(methodName) && method.getParameterTypes().length == 1)
                        {
                            if (method.getParameterTypes()[0].equals(boolean.class))
                            {
                                method.invoke(OtsTransceiver.this.tacticalFactory, (boolean) value);
                            }
                            else if (method.getParameterTypes()[0].equals(int.class))
                            {
                                method.invoke(OtsTransceiver.this.tacticalFactory, (int) value);
                            }
                            else if (method.getParameterTypes()[0].equals(double.class))
                            {
                                method.invoke(OtsTransceiver.this.tacticalFactory, (double) value);
                            }
                            else if (method.getParameterTypes()[0].isEnum())
                            {
                                @SuppressWarnings("unchecked")
                                Class<T> clazz = (Class<T>) method.getParameterTypes()[0];
                                method.invoke(OtsTransceiver.this.tacticalFactory, Enum.valueOf(clazz, (String) value));
                            }
                            else
                            {
                                CategoryLogger.always().warn("Unable to set parameter " + parameter);
                                break;
                            }
                        }
                    }
                }
                else
                {
                    setParameterValue(parameter, value, setParameters);
                }
            }
            /*-
             * Method notify() needs a GTU inside planGtuIds to send an OperationPlan to ExternalSim. For different modes:
             *  Ots: register here, spawn (plan is sent), schedule control mode change (without effect)
             *  Hybrid: spawn, schedule control mode change (starts dead reckoning, plan is sent)
             *  External: spawn, schedule control mode change (starts dead reckoning, never sent plan)
             * Note that dead reckoning can only be started after a spawn.
             */
            if (mode.toLowerCase().equals("ots"))
            {
                this.planGtuIds.add(id);
            }
            this.gtuSpawner.spawnGtu(id, gtuType, vehicleLength, vehicleWidth, refToNose, route, initSpeed, position);
            OtsTransceiver.this.tacticalFactory.resetMode();
            setParameters.forEach((p) -> this.parameterFactory.clearParameterValue(p));
            scheduledChangeControlMode(id, mode);
        }

        /**
         * Add active mode object.
         * @param id id
         * @param location location
         * @param speed speed
         * @throws NetworkException no valid location
         */
        private void addActiveModeObject(final String id, final OrientedPoint2d location, final Speed speed)
                throws NetworkException
        {
            LanePosition position = NetworkUtil.getLanePosition(this.network, location);
            ActiveModeCrossing crossing = new ActiveModeCrossing(position, false);
            this.activeIds.put(id, crossing);
            updateActiveModeObject(id, location, speed);
        }

        /**
         * Updates crossing arrival time of active mode object.
         * @param id id
         * @param location location
         * @param speed speed
         */
        private void updateActiveModeObject(final String id, final OrientedPoint2d location, final Speed speed)
        {
            ActiveModeCrossing crossing = this.activeIds.get(id);
            Length width = crossing.getLane().getWidth(crossing.getLongitudinalPosition());
            Length distance = Length.instantiateSI(Math.max(location.distance(crossing.getLocation()) - width.si / 2.0, 0.0));
            crossing.setArrival(id, distance, speed);
        }

        /**
         * Sets parameter in parameter factory.
         * @param <T> value type
         * @param parameter parameter type id
         * @param value value
         * @param setParameters set of parameters that are set, to which the parameter should be added
         */
        @SuppressWarnings("unchecked")
        private <T> void setParameterValue(final String parameter, final Object value,
                final Set<ParameterType<?>> setParameters)
        {
            ParameterType<T> param = (ParameterType<T>) Parameters.get(parameter);
            this.parameterFactory.setParameterValue(param, (T) value);
            setParameters.add(param);
        }

        /**
         * Scheduled delete running in the simulator.
         * @param id GTU id
         */
        @SuppressWarnings("unused") // scheduled
        private void scheduledDelete(final String id)
        {
            this.network.getGTU(id).destroy();
        }

        /**
         * Method that runs scheduled in the simulator to apply dead reckoning.
         * @param id GTU id
         * @param loc location
         * @param speed speed
         * @param acceleration acceleration
         */
        @SuppressWarnings("unused") // scheduled
        private void scheduledDeadReckoning(final String id, final OrientedPoint2d loc, final Speed speed,
                final Acceleration acceleration)
        {
            ScenarioTacticalPlanner planner = getTacticalPlanner(id);
            if (planner != null)
            {
                planner.deadReckoning(loc, speed, acceleration);
            }
        }

        /**
         * Perform command on GTU.
         * @param id GTU id
         * @param json JSON string of command
         */
        @SuppressWarnings("unused") // scheduled
        private void scheduledPerformCommand(final String id, final String json)
        {
            Commands.Command command = this.gson.fromJson(json, Commands.Command.class);
            Function<String, CommandsHandler> function =
                    (gtuId) -> new CommandsHandler(this.network, new Commands(gtuId, null), null);
            this.commandHandlers.computeIfAbsent(id, function).scheduleCommand(command);
        }

        /**
         * Initializes or changes the control mode of the GTU with given id.
         * @param id GTU id
         * @param mode control mode, Ots, Hybrid or External
         */
        private void scheduledChangeControlMode(final String id, final String mode)
        {
            switch (mode.toLowerCase())
            {
                case "ots":
                {
                    this.planGtuIds.add(id);
                    if (this.externalGtuIds.remove(id))
                    {
                        getTacticalPlanner(id).stopDeadReckoning();
                    }
                    break;
                }
                case "hybrid":
                {
                    this.planGtuIds.add(id);
                    if (this.externalGtuIds.add(id))
                    {
                        getTacticalPlanner(id).startDeadReckoning(true);
                    }
                    break;
                }
                case "external":
                {
                    this.planGtuIds.remove(id);
                    if (this.externalGtuIds.add(id))
                    {
                        getTacticalPlanner(id).startDeadReckoning(false);
                    }
                    break;
                }
                default:
                    System.err.println("Unknown control mode " + mode);
                    break;
            }
        }

        /**
         * Notifies the external simulator that OTS is ready to start a simulation.
         * @param msgId message id of message that was processed
         * @param queued whether the message should be queued
         * @throws RemoteException
         */
        private void sentReadyMessage(final int msgId, final boolean queued) throws RemoteException
        {
            try
            {
                byte[] bytes = Sim0MQMessage.encodeUTF8(OtsTransceiver.this.bigEndian, OtsTransceiver.this.federation,
                        OtsTransceiver.this.ots, OtsTransceiver.this.client, "READY", this.messageId++, new Object[] {msgId});
                if (queued)
                {
                    this.queue.add(new QueuedMessage(bytes, String.format("[%.3fs] Ots sent READY message for PROGRESS (%d)",
                            this.simulator.getSimulatorTime().si, msgId)));
                }
                else
                {
                    this.responder.send(bytes, ZMQ.DONTWAIT);
                }
            }
            catch (Sim0MQException | SerializationException ex)
            {
                throw new RemoteException("Exception while sending operational plan.", ex);
            }
        }

        /**
         * Returns the scenario based tactical planner for GTU with given id.
         * @param gtuId GTU id
         * @return scenario based tactical planner for GTU with given id, {@code null} if the vehicle does not exist
         */
        private ScenarioTacticalPlanner getTacticalPlanner(final String gtuId)
        {
            Gtu gtu = this.network.getGTU(gtuId);
            if (gtu == null)
            {
                return null;
            }
            return (ScenarioTacticalPlanner) gtu.getTacticalPlanner();
        }

        /**
         * Stops any running simulation. It leaves data to be able to reset the simulation.
         */
        private void stopSimulation()
        {
            if (this.simulator != null && !this.simulator.isStoppingOrStopped())
            {
                this.simulator.stop();
                this.simulator = null;
                this.network = null;
            }
            this.planGtuIds.clear();
            this.externalGtuIds.clear();
            this.commandHandlers.clear();
            if (this.app != null)
            {
                this.app.dispose();
                this.app = null;
            }
        }

        /**
         * Clears the information that is used to setup a simulation. This is the information that would be reused in a reset
         * message.
         */
        private void clearSimulationSetupData()
        {
            this.lastNetworkMessage = null;
            this.lastOdJson = null;
            this.lastRoutesJson = null;
            this.preStartVehiclePayloads.clear();
        }

        /**
         * Setup simulation.
         * @throws GtuException exception
         * @throws OtsGeometryException exception
         * @throws NetworkException exception
         * @throws RemoteException exception
         * @throws DsolException exception
         * @throws OtsDrawingException exception
         * @throws SimRuntimeException exception
         * @throws NamingException exception
         * @throws ParserConfigurationException exception
         * @throws SAXException exception
         * @throws JAXBException exception
         * @throws InvocationTargetException if a parameter cannot be set
         * @throws IllegalAccessException if a parameter cannot be set
         */
        private void setupSimulation() throws GtuException, OtsGeometryException, NetworkException, RemoteException,
                DsolException, OtsDrawingException, SimRuntimeException, NamingException, ParameterException, JAXBException,
                SAXException, ParserConfigurationException, IllegalAccessException, InvocationTargetException
        {
            stopSimulation();

            if (this.lastNetworkMessage == null)
            {
                return;
            }

            // An animator supports real-time running. No GUI will be shown if no animation panel is created.
            this.simulator = new OtsAnimator("Test animator");
            // this.simulator.addListener(this, SimulatorInterface.STOP_EVENT);
            this.simulator.addListener(this, PROGRESSED_EVENT);

            String simulationString;
            SimulationType simulationType;
            Object[] payload = this.lastNetworkMessage.createObjectArray();
            if ((short) payload[7] == 0 || payload[8] == null || (payload[8] instanceof String str && str.isBlank())
                    || !(payload[8] instanceof String))
            {
                // No network payload: demo co-simulation model
                simulationString = null;
                simulationType = null;
            }
            else
            {
                simulationString = (String) payload[8];
                // TODO default is now OPEN_DRIVE within the i4Driving context, remove default in OTS context
                simulationType =
                        (short) payload[7] > 1 ? SimulationType.valueOf((String) payload[9]) : SimulationType.OPEN_DRIVE;
            }
            CoSimModel model = new CoSimModel(this.simulator, simulationString, simulationType);
            Duration runtime = simulationType == null ? Duration.instantiateSI(60.0) : Duration.instantiateSI(36000.0);
            this.simulator.initialize(Time.ZERO, Duration.ZERO, runtime, model);
            this.simulator.getReplication().setHistoryManager(
                    new HistoryManagerDevs(this.simulator, Duration.instantiateSI(5.0), Duration.instantiateSI(10.0)));
            this.network = (RoadNetwork) model.getNetwork();
            this.characteristicsGeneratorOd = model.getSim0mqSimulation().getGtuCharacteristicsGeneratorOd();
            this.parameterFactory = model.getSim0mqSimulation().getParameterFactory();
            this.gtuSpawner = new GtuSpawnerOd(this.network, this.characteristicsGeneratorOd);

            listenToEvents();

            if (OtsTransceiver.this.showGui)
            {
                GtuColorer colorer = new SwitchableGtuColorer(0, new IdGtuColorer(),
                        new SpeedGtuColorer(new Speed(150, SpeedUnit.KM_PER_HOUR)),
                        new AccelerationGtuColorer(Acceleration.instantiateSI(-6.0), Acceleration.instantiateSI(2)),
                        new SynchronizationColorer());
                OtsAnimationPanel animationPanel = new OtsAnimationPanel(this.network.getExtent(), new Dimension(100, 100),
                        this.simulator, model, colorer, this.network);
                animationPanel.enableSimulationControlButtons();
                this.app = new OtsSimulationApplication<AbstractOtsModel>(model, animationPanel);
            }

            // Routes
            if (this.lastRoutesJson != null)
            {
                this.lastRoutesJson.createRoutes(this.network, DefaultsNl.VEHICLE, model.getSim0mqSimulation());
            }

            // OD background traffic with default model
            if (this.lastOdJson != null)
            {
                Collection<GtuType> gtuTypes = Set.of(DefaultsNl.CAR, DefaultsNl.VAN, DefaultsNl.BUS, DefaultsNl.TRUCK);
                OdMatrix od = this.lastOdJson.asOdMatrix(this.network, gtuTypes, model.getSim0mqSimulation());
                OdOptions odOptions = new OdOptions().set(OdOptions.GTU_TYPE, this.characteristicsGeneratorOd)
                        .set(OdOptions.GTU_ID, new IdGenerator(OtsTransceiver.this.idPrefix));
                OdApplier.applyOd(this.network, od, odOptions, DefaultsRoadNl.ROAD_USERS);
            }

            // Reset pre-start vehicles
            for (Object[] vehiclesPayload : this.preStartVehiclePayloads)
            {
                generateVehicle(vehiclesPayload, false); // do not add these messages to the list
            }

        }

        @Override
        public void notify(final Event event) throws RemoteException
        {
            EventType eventType = event.getType();
            if (eventType.equals(LaneBasedGtu.LANEBASED_MOVE_EVENT))
            {
                String gtuId = (String) ((Object[]) event.getContent())[0];
                if (!this.planGtuIds.contains(gtuId))
                {
                    return;
                }
                /*
                 * When the GTU is in hybrid mode, it sends a plan every so often. ScenarioTacticalPlanner will schedule a
                 * second move event at the same time that will trigger the dead-reckoning to continue until we receive new
                 * dead-reckoning information.
                 */
                sendOperationalPlanMessage(gtuId);
            }
            else if (eventType.equals(Network.GTU_ADD_EVENT))
            {
                String gtuId = (String) event.getContent();
                Gtu gtu = this.network.getGTU(gtuId);
                if (!gtuId.equals(this.externallyGeneratedGtuId))
                {
                    this.simulator.scheduleEventNow(this, "sendVehicleMessage", new Object[] {gtu});
                    this.planGtuIds.add(gtuId);
                    this.externallyGeneratedGtuId = null;
                }
                gtu.addListener(this, LaneBasedGtu.LANEBASED_MOVE_EVENT);
            }
            else if (eventType.equals(Network.GTU_REMOVE_EVENT))
            {
                String gtuId = (String) event.getContent();
                Gtu gtu = this.network.getGTU(gtuId);
                gtu.removeListener(this, LaneBasedGtu.LANEBASED_MOVE_EVENT);
                this.planGtuIds.remove(gtuId);
                this.externalGtuIds.remove(gtuId);
                ActiveModeCrossing crossing = this.activeIds.remove(gtuId);
                if (crossing != null)
                {
                    crossing.removeArrival(gtuId);
                }
                this.commandHandlers.remove(gtuId);
                if (!this.deleteGtuIds.remove(gtuId))
                {
                    sendDeleteMessage(gtuId);
                }
            }
            else if (eventType.equals(SimulatorInterface.STOP_EVENT))
            {
                if (this.runUntil != null) // && this.simulator.getSimulatorTime().eq(this.runUntil))
                {
                    this.runUntil = null;
                    sentReadyMessage(this.progressMessageId, true);
                }
                // if not, stopped for some other reason, perhaps a stop button in the GUI
            }
            else if (eventType.equals(PROGRESSED_EVENT))
            {
                if (this.runUntil == null)
                {
                    System.err.println("PROGRESSED_EVENT but no runUntil value");
                }
                this.runUntil = null;
                sentReadyMessage(this.progressMessageId, true);
            }
        }

        /**
         * Sent VEHICLE message.
         * @param gtu
         * @throws Sim0MQException
         * @throws SerializationException
         */
        @SuppressWarnings("unused") // scheduled
        private void sendVehicleMessage(final Gtu gtu) throws Sim0MQException, SerializationException
        {
            String gtuId = gtu.getId();
            OrientedPoint2d p = gtu.getLocation();
            String routeId = gtu.getStrategicalPlanner().getRoute().getId();
            Object[] payload = new Object[] {gtuId, "Ots", Length.instantiateSI(p.x), Length.instantiateSI(p.y),
                    Direction.instantiateSI(p.dirZ), gtu.getSpeed(), gtu.getType().getId(), gtu.getLength(), gtu.getWidth(),
                    gtu.getFront().dx(), 0, routeId};

            byte[] bytes = Sim0MQMessage.encodeUTF8(OtsTransceiver.this.bigEndian, OtsTransceiver.this.federation,
                    OtsTransceiver.this.ots, OtsTransceiver.this.client, "VEHICLE", this.messageId++, payload);
            String log = String.format("[%.3fs] Ots sent VEHICLE message for GTU %s on route %s",
                    this.simulator.getSimulatorTime().si, gtuId, routeId);
            this.queue.add(new QueuedMessage(bytes, log));
        }

        /**
         * Send operational plan.
         * @param gtuId GTU id
         * @throws RemoteException exception
         */
        private void sendOperationalPlanMessage(final String gtuId) throws RemoteException
        {
            Gtu gtu = this.network.getGTU(gtuId);
            OperationalPlan plan = ((ScenarioTacticalPlanner) gtu.getTacticalPlanner()).pullLastIntendedPlan();
            if (plan == null)
            {
                // Do not sent plan upon a move triggered by external control
                return;
            }
            Speed speed = plan.getStartSpeed();
            OtsLine2d line = plan.getPath();
            float[] x = new float[line.size()];
            float[] y = new float[line.size()];
            try
            {
                for (int i = 0; i < line.size(); i++)
                {
                    x[i] = (float) line.get(i).x;
                    y[i] = (float) line.get(i).y;
                }
            }
            catch (OtsGeometryException ex)
            {
                throw new RemoteException("Unable to obtain coordinate from path.", ex);
            }
            ImmutableList<Segment> segments = plan.getOperationalPlanSegmentList();
            float[] t = new float[segments.size()];
            float[] a = new float[segments.size()];
            for (int i = 0; i < segments.size(); i++)
            {
                t[i] = (float) segments.get(i).duration().si;
                a[i] = (float) segments.get(i).acceleration().si;
            }

            Object[] payload = new Object[7];
            payload[0] = gtuId;
            payload[1] = speed;
            payload[2] = new FloatLengthVector(x);
            payload[3] = new FloatLengthVector(y);
            payload[4] = new FloatDurationVector(t);
            payload[5] = new FloatAccelerationVector(a);
            switch (((LaneBasedGtu) gtu).getTurnIndicatorStatus())
            {
                case LEFT:
                    payload[6] = "Left";
                    break;
                case RIGHT:
                    payload[6] = "Right";
                    break;
                case NONE:
                    payload[6] = "None";
                    break;
                case HAZARD:
                    payload[6] = "Both";
                    break;
                default:
                    payload[6] = "None";
            }

            try
            {
                byte[] bytes = Sim0MQMessage.encodeUTF8(OtsTransceiver.this.bigEndian, OtsTransceiver.this.federation,
                        OtsTransceiver.this.ots, OtsTransceiver.this.client, "PLAN", this.messageId++, payload);
                String log = String.format("[%.3fs] Ots sent PLAN message for GTU %s (a=%.3fm/s^2)",
                        this.simulator.getSimulatorTime().si, gtuId, a[0]);
                this.queue.add(new QueuedMessage(bytes, log));
            }
            catch (Sim0MQException | SerializationException ex)
            {
                throw new RemoteException("Exception while sending operational plan.", ex);
            }
        }

        /**
         * Send delete message to external sim.
         * @param gtuId GTU id
         * @throws RemoteException exception
         */
        private void sendDeleteMessage(final String gtuId) throws RemoteException
        {
            Object[] payload = new Object[1];
            payload[0] = gtuId;
            try
            {
                byte[] bytes = Sim0MQMessage.encodeUTF8(OtsTransceiver.this.bigEndian, OtsTransceiver.this.federation,
                        OtsTransceiver.this.ots, OtsTransceiver.this.client, "DELETE", this.messageId++, payload);
                String log = String.format("[%.3fs] Ots sent DELETE message for GTU %s", this.simulator.getSimulatorTime().si,
                        gtuId);
                this.queue.add(new QueuedMessage(bytes, log));
            }
            catch (Sim0MQException | SerializationException ex)
            {
                throw new RemoteException("Exception while sending operational plan.", ex);
            }
        }

        /**
         * Listen to relevant events.
         * @throws RemoteException exception
         */
        private void listenToEvents() throws RemoteException
        {
            this.network.addListener(this, Network.GTU_ADD_EVENT);
            this.network.addListener(this, Network.GTU_REMOVE_EVENT);
        }

        /**
         * Fire PROGRESSED_EVENT.
         */
        private void fireProgressedEvent()
        {
            this.simulator.fireEvent(PROGRESSED_EVENT);
        }
    }

    /**
     * Co-simulation model. This intermediates between an OTS model, and the different supported network/OD types.
     */
    private final class CoSimModel extends AbstractOtsModel
    {
        /** */
        private static final long serialVersionUID = 1L;

        /** String definition of the network. */
        private final String simulationString;

        /** Network type. */
        private final SimulationType simulationType;

        /** Simulation. */
        private Sim0mqSimulation simulation;

        /**
         * Constructor.
         * @param simulator simulator
         * @param simulationString network string
         * @param simulationType network type
         */
        private CoSimModel(final OtsSimulatorInterface simulator, final String simulationString,
                final SimulationType simulationType)
        {
            super(simulator);
            this.simulationString = simulationString;
            this.simulationType = simulationType;
        }

        @Override
        public Network getNetwork()
        {
            return this.simulation.getNetwork();
        }

        /**
         * Returns the sim0mq simulation.
         * @return sim0mq simulation
         */
        public Sim0mqSimulation getSim0mqSimulation()
        {
            return this.simulation;
        }

        @Override
        public void constructModel() throws SimRuntimeException
        {
            try
            {
                if (this.simulationString == null)
                {
                    this.simulation = new TwoLaneTestSimulation(getSimulator(), OtsTransceiver.this.tacticalFactory);
                }
                else
                {
                    switch (this.simulationType)
                    {
                        case OPEN_DRIVE:
                            this.simulation = new OpenDriveSimulation(this.simulator, OtsTransceiver.this.tacticalFactory,
                                    this.simulationString, OtsTransceiver.this.useRoadName);
                            break;
                        case FOSIM:
                            // TODO parse Fosim string
                            // break;
                        case OTS:
                            // TODO parse OTS string
                            // break;
                        default:
                            throw new SimRuntimeException("Network type " + this.simulationType + " is not supported.");
                    }
                }
            }
            catch (GtuException | OtsGeometryException | NetworkException | JAXBException | SAXException
                    | ParserConfigurationException ex)
            {
                throw new SimRuntimeException(ex);
            }
        }

    }

    /** All the converters that decode into arrays and matrices of Objects, keyed by prefix. */
    private static final Map<Byte, Serializer<?>> OBJECT_DECODERS = new HashMap<>();

    static
    {
        try
        {
            Field field = TypedMessage.class.getDeclaredField("OBJECT_DECODERS");
            field.setAccessible(true);
            @SuppressWarnings("unchecked")
            Map<Byte, Serializer<?>> map = (Map<Byte, Serializer<?>>) field.get(TypedMessage.class);
            map.forEach((b, s) ->
            {
                OBJECT_DECODERS.put(b, s);
                OBJECT_DECODERS.put((byte) (b - 128), s);
            });
        }
        catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e)
        {
            e.printStackTrace();
        }
    }

    /**
     * Queued messages.
     * @param message bytes of the message
     * @param log log entry to print when the message is sent
     */
    private record QueuedMessage(byte[] message, String log)
    {
    }

    /**
     * Simulation type, defining how the simulation string containing the network and optionally demand is parsed.
     */
    private enum SimulationType
    {
        /** OpenDRIVE network with optional separate JSON OD matrix. */
        OPEN_DRIVE,

        /** FOSIM simulation file. */
        FOSIM,

        /** OTS XML simulation. */
        OTS;
    }

    /** Event when simulator has ran up to a time. */
    EventType PROGRESSED_EVENT = new EventType(new MetaData("PROGRESSED", "Simulation ran up to PROGRESS time."));

}
