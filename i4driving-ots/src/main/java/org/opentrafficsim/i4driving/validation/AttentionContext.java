package org.opentrafficsim.i4driving.validation;

import java.awt.Color;

import org.djunits.value.vdouble.scalar.Length;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.network.RoadNetwork;

public interface AttentionContext
{

    RoadNetwork getNetwork();

    String getGtuId();

    int getNumberOfGroups();

    int getGroupIndex(Object channel);

    int getGroupIndexByLabel(String groupLabel);

    String getGroupLabel(int groupIndex);

    Color getGroupColor(int groupIndex);
    
    Color getIdleColor();
    
    Color getAnticipationRelianceColor();

    default double getBinSize()
    {
        return 1.0;
    }

    Length getZeroOdometer(LaneBasedGtu gtu);
    
    Length getMaxDistance(LaneBasedGtu gtu);

}
