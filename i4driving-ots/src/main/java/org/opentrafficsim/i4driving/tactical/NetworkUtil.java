package org.opentrafficsim.i4driving.tactical;

import org.djutils.draw.point.Point2d;
import org.opentrafficsim.core.geometry.OtsLine2d.FractionalFallback;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.LanePosition;

/**
 * Network utilities.
 * @author wjschakel
 */
public final class NetworkUtil
{

    /**
     * Constructor.
     */
    private NetworkUtil()
    {
        //
    }

    /**
     * Returns the lane position closest to the given location.
     * @param network network
     * @param position position
     * @return lane position closest to the given location
     */
    public static LanePosition getLanePosition(final RoadNetwork network, final Point2d position)
    {
        double minDistance = Double.POSITIVE_INFINITY;
        LanePosition lanePosition = null;
        for (Link link : network.getLinkMap().values())
        {
            if (link instanceof CrossSectionLink roadLink)
            {
                for (Lane lane : roadLink.getLanesAndShoulders())
                {
                    double fraction = lane.getCenterLine().projectFractional(link.getStartNode().getHeading(),
                            link.getEndNode().getHeading(), position.x, position.y, FractionalFallback.ENDPOINT);
                    fraction = fraction < 0.0 ? 0.0 : (fraction > 1.0 ? 1.0 : fraction);
                    Point2d pointOnLane = lane.getCenterLine().getLocationFractionExtended(fraction);
                    double distance = pointOnLane.distance(position);
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        lanePosition = new LanePosition(lane, lane.getCenterLine().getLength().times(fraction));
                    }
                }
            }
        }
        return lanePosition;
    }

}
