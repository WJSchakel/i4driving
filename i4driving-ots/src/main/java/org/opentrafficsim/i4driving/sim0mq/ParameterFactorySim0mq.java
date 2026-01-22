package org.opentrafficsim.i4driving.sim0mq;

import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.djutils.exceptions.Throw;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterType;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;

/**
 * Extension of parameter factory that allows temporary setting and resetting of parameter drawn.
 * @author wjschakel
 */
public class ParameterFactorySim0mq extends ParameterFactoryByType
{

    /** Overwriting parameters. */
    private final Map<ParameterType<?>, Object> values = new LinkedHashMap<>();

    /**
     * Sets overwriting parameter value.
     * @param <T> value type
     * @param parameter parameter
     * @param value value
     */
    public <T> void setParameterValue(final ParameterType<T> parameter, final T value)
    {
        this.values.put(parameter, value);
    }

    /**
     * Clears overwriting parameter value.
     * @param parameter parameter
     */
    public void clearParameterValue(final ParameterType<?> parameter)
    {
        this.values.remove(parameter);
    }

    /**
     * Clear all parameters set with {@link #setParameterValue(ParameterType, Object)}.
     */
    public void clearSetParameters()
    {
        this.values.clear();
    }

    @Override
    public void setValues(final Parameters parameters, final GtuType gtuType) throws ParameterException
    {
        super.setValues(parameters, gtuType);
        setValues(parameters);
    }

    @SuppressWarnings("unchecked")
    private <T> void setValues(final Parameters parameters) throws ParameterException
    {
        Set<Entry<ParameterType<?>, Object>> vals = new LinkedHashSet<>(this.values.entrySet());
        while (vals.size() > 0)
        {
            int preSize = vals.size();
            Iterator<Entry<ParameterType<?>, Object>> it = vals.iterator();
            while (it.hasNext())
            {
                Entry<ParameterType<?>, Object> entry = it.next();
                try
                {
                    parameters.setParameter((ParameterType<T>) entry.getKey(), (T) entry.getValue());
                    it.remove();
                }
                catch (ParameterException ex)
                {
                    // try next loop, we might first have to set another parameter for a constraint to hold
                }
            }
            Throw.when(vals.size() == preSize, ParameterException.class,
                    "Unable to set parameters due to contrains violation: %s", vals);
        }
    }

}
