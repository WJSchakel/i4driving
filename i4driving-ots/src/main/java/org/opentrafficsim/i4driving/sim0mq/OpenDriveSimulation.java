package org.opentrafficsim.i4driving.sim0mq;

import java.util.LinkedHashMap;
import java.util.Map;

import javax.xml.bind.JAXBException;
import javax.xml.parsers.ParserConfigurationException;

import org.opentrafficsim.core.definitions.Defaults;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.geometry.OtsGeometryException;
import org.opentrafficsim.core.gtu.GtuCharacteristics;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.i4driving.opendrive.OpenDriveParser;
import org.opentrafficsim.i4driving.tactical.ScenarioTacticalPlannerFactory;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuCharacteristics;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuCharacteristicsGeneratorOd;
import org.opentrafficsim.road.gtu.lane.VehicleModel;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalRoutePlannerFactory;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.od.Category;
import org.xml.sax.SAXException;

import nl.tudelft.simulation.jstats.streams.StreamInterface;

/**
 * OpenDRIVE simulation.
 * @author wjschakel
 */
public class OpenDriveSimulation implements Sim0mqSimulation
{

    /** Network. */
    private final RoadNetwork network;

    /** GTU characteristics generator. */
    private final LaneBasedGtuCharacteristicsGeneratorOd charateristicsGeneratorOd;

    /** Parameter factory. */
    private ParameterFactorySim0mq parameterFactory;

    /** Registered GTU types. */
    private final Map<GtuType, GtuCharacteristics> registeredGtuTypes = new LinkedHashMap<>();

    /** Parser. */
    private final OpenDriveParser parser;

    /**
     * Constructor.
     * @param simulator simulator
     * @param tacticalFactory tactical planner factory
     * @param networkString OpenDRIVE string
     * @param useRoadName whether to use the road name to identify origins and destinations
     * @throws JAXBException
     * @throws SAXException
     * @throws ParserConfigurationException
     * @throws NetworkException
     * @throws OtsGeometryException
     * @throws GtuException
     */
    public OpenDriveSimulation(final OtsSimulatorInterface simulator, final ScenarioTacticalPlannerFactory tacticalFactory,
            final String networkString, final boolean useRoadName) throws JAXBException, SAXException,
            ParserConfigurationException, NetworkException, OtsGeometryException, GtuException
    {
        this.parser = OpenDriveParser.parseFileString(networkString).setUseRoadName(useRoadName);
        this.network = new RoadNetwork("OtsOpenDriveNetwork", simulator);
        this.parser.build(this.network);
        OpenDriveParser.buildConflicts(this.network);

        // Model
        StreamInterface stream = simulator.getModel().getStream("generation");
        tacticalFactory.setStream(stream);
        // LaneBasedTacticalPlannerFactory<?> tacticalFactory = mixinModel.getTacticalPlanner(stream);
        this.parameterFactory = new ParameterFactorySim0mq();

        // GTU characteristics generator
        GtuType.registerTemplateSupplier(DefaultsNl.CAR, Defaults.NL);
        GtuCharacteristics gtucharacteristicsCar = GtuType.defaultCharacteristics(DefaultsNl.CAR, this.network, stream);
        this.registeredGtuTypes.put(DefaultsNl.CAR, gtucharacteristicsCar);
        LaneBasedStrategicalRoutePlannerFactory stratFactory =
                new LaneBasedStrategicalRoutePlannerFactory(tacticalFactory, this.parameterFactory);
        this.charateristicsGeneratorOd = new LaneBasedGtuCharacteristicsGeneratorOd()
        {
            @Override
            public LaneBasedGtuCharacteristics draw(final Node origin, final Node destination, final Category category,
                    final StreamInterface randomStream) throws GtuException
            {
                GtuType gtuType =
                        category.getCategorization().entails(GtuType.class) ? category.get(GtuType.class) : DefaultsNl.CAR;
                if (!OpenDriveSimulation.this.registeredGtuTypes.containsKey(gtuType))
                {
                    GtuType.registerTemplateSupplier(gtuType, Defaults.NL);
                    OpenDriveSimulation.this.registeredGtuTypes.put(gtuType,
                            GtuType.defaultCharacteristics(DefaultsNl.CAR, OpenDriveSimulation.this.network, stream));
                }
                GtuCharacteristics gtucharacteristics = OpenDriveSimulation.this.registeredGtuTypes.get(gtuType);

                Route route = category.getCategorization().entails(Route.class) ? category.get(Route.class) : null;

                return new LaneBasedGtuCharacteristics(gtucharacteristics, stratFactory, route, origin, destination,
                        VehicleModel.MINMAX);
            }
        };
    }

    @Override
    public RoadNetwork getNetwork()
    {
        return this.network;
    }

    @Override
    public LaneBasedGtuCharacteristicsGeneratorOd getGtuCharacteristicsGeneratorOd()
    {
        return this.charateristicsGeneratorOd;
    }

    @Override
    public ParameterFactorySim0mq getParameterFactory()
    {
        return this.parameterFactory;
    }

    @Override
    public String getOrigin(final String id, final Boolean designDirection)
    {
        return this.parser.getOrigin(id, designDirection).getId();
    }

    @Override
    public String getDestination(final String id, final Boolean designDirection)
    {
        return this.parser.getDestination(id, designDirection).getId();
    }

    @Override
    public RouteObjectType getRouteObjectType()
    {
        return RouteObjectType.ROAD;
    }

}
