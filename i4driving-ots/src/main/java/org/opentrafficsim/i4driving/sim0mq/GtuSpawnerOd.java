package org.opentrafficsim.i4driving.sim0mq;

import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djutils.draw.point.Point2d;
import org.djutils.multikeymap.MultiKeyMap;
import org.opentrafficsim.core.geometry.OtsGeometryException;
import org.opentrafficsim.core.gtu.GtuCharacteristics;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.i4driving.tactical.NetworkUtil;
import org.opentrafficsim.road.gtu.generator.GtuSpawner;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuCharacteristics;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuCharacteristicsGeneratorOd;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.od.Categorization;
import org.opentrafficsim.road.od.Category;

/**
 * GTU spawner based on VEHICLE message.
 * @author wjschakel
 */
public class GtuSpawnerOd
{

    /** Low-level GTU spawner. */
    private final GtuSpawner gtuSpawner = new GtuSpawner();

    /** Network. */
    private final RoadNetwork network;

    /** GTU characteristics generator from simulation. */
    private final LaneBasedGtuCharacteristicsGeneratorOd characteristicsGenerator;

    /** Categorization based on GTU type and route, i.e. VEHICLE message. */
    private final Categorization categorization = new Categorization("SpawnCategorization", GtuType.class, Route.class);

    /** Stored categories. */
    private final MultiKeyMap<Category> categories = new MultiKeyMap<>(GtuType.class, Route.class);

    public GtuSpawnerOd(final RoadNetwork network, final LaneBasedGtuCharacteristicsGeneratorOd characteristicsGenerator)
    {
        this.network = network;
        this.characteristicsGenerator = characteristicsGenerator;
    }

    /**
     * Constructor.
     * @param id id
     * @param gtuType GTU type
     * @param length length
     * @param width width
     * @param front distance from reference point to front
     * @param route route
     * @param speed speed
     * @param position position
     * @throws GtuException when initial GTU values are not correct
     * @throws OtsGeometryException when the initial path is wrong
     * @throws NetworkException when the GTU cannot be placed on the given position
     */
    @SuppressWarnings("parameternumber")
    public void spawnGtu(final String id, final GtuType gtuType, final Length length, final Length width, final Length front,
            final Route route, final Speed speed, final Point2d position)
            throws GtuException, OtsGeometryException, NetworkException
    {
        LaneBasedGtuCharacteristics standardTemplate =
                this.characteristicsGenerator.draw(route.originNode(), route.destinationNode(), getCategory(gtuType, route),
                        this.network.getSimulator().getModel().getStream("generation"));
        GtuCharacteristics overwrittenBaseCharacteristics =
                new GtuCharacteristics(gtuType, length, width, standardTemplate.getMaximumSpeed(),
                        standardTemplate.getMaximumAcceleration(), standardTemplate.getMaximumDeceleration(), front);
        LaneBasedGtuCharacteristics templateGtuType = new LaneBasedGtuCharacteristics(overwrittenBaseCharacteristics,
                standardTemplate.getStrategicalPlannerFactory(), route, standardTemplate.getOrigin(),
                standardTemplate.getDestination(), standardTemplate.getVehicleModel());
        this.gtuSpawner.spawnGtu(id, templateGtuType, this.network, speed, NetworkUtil.getLanePosition(this.network, position));
    }

    /**
     * Returns category. It is either created, or taken from memory.
     * @param gtuType GTU type
     * @param route route
     * @return category
     */
    private Category getCategory(final GtuType gtuType, final Route route)
    {
        return this.categories.get(() -> new Category(this.categorization, gtuType, route), gtuType, route);
    }

}
