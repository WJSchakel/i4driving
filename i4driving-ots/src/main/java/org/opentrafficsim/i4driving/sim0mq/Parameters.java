package org.opentrafficsim.i4driving.sim0mq;

import java.lang.reflect.Field;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import org.djutils.exceptions.Throw;
import org.djutils.reflection.ClassUtil;
import org.opentrafficsim.base.parameters.ParameterType;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.i4driving.tactical.perception.mental.CarFollowingTask;
import org.opentrafficsim.i4driving.tactical.perception.mental.TaskManagerAr;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelFuller;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskScan;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelTaskSignal;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationHeadway;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSpeed;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;

/**
 * Supported parameters.
 */
public final class Parameters
{

    /**
     * Constructor.
     */
    private Parameters()
    {
        //
    }

    /** Map of parameter types by their id. */
    private static final Map<String, ParameterType<?>> MAP = new LinkedHashMap<>();

    /**
     * Add parameter type to map.
     * @param parameterType parameter type
     */
    private static void add(final ParameterType<?> parameterType)
    {
        MAP.put(parameterType.getId(), parameterType);
    }

    /**
     * Add all parameter types of class.
     * @param clazz class with static parameter types
     */
    private static void add(final Class<?> clazz)
    {
        // add all parameter types using reflection
        Set<Field> fields = ClassUtil.getAllFields(clazz);

        for (Field field : fields)
        {
            if (ParameterType.class.isAssignableFrom(field.getType()))
            {
                try
                {
                    field.setAccessible(true);
                    add((ParameterType<?>) field.get(clazz));
                }
                catch (IllegalArgumentException | IllegalAccessException ex)
                {
                    // should not happen, field and clazz are related
                    throw new RuntimeException(ex);
                }
            }
        }
    }

    static
    {
        add(ParameterTypes.class);
        add(LmrsParameters.class);
        add(Fuller.class);
        add(TaskManagerAr.class);
        add(CarFollowingTask.class);
        add(AdaptationHeadway.class);
        add(AdaptationSpeed.class);
        add(ChannelFuller.class);
        add(ChannelTaskScan.class);
        add(ChannelTaskSignal.class);
        MAP.put("x0", ParameterTypes.LOOKAHEAD);
    }

    /**
     * Prints all supported parameters.
     * @param args
     */
    public static void main(final String[] args)
    {
        MAP.forEach((id, p) -> System.out
                .println("\"" + id + "\", \"" + p.getValueClass().getSimpleName() + "\", \"" + p.getDescription() + "\""));
    }

    /**
     * Get parameter type by id.
     * @param parameterTypeId parameter type id
     * @return parameter type
     */
    public static ParameterType<?> get(final String parameterTypeId)
    {
        Throw.when(!MAP.containsKey(parameterTypeId), IllegalArgumentException.class, "Parameter %s is not supported.",
                parameterTypeId);
        return MAP.get(parameterTypeId);
    }

}
