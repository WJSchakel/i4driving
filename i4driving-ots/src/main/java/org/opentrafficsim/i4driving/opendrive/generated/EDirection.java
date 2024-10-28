
package org.opentrafficsim.i4driving.opendrive.generated;

import javax.xml.bind.annotation.XmlEnum;
import javax.xml.bind.annotation.XmlEnumValue;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for e_direction.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * <pre>
 * &lt;simpleType name="e_direction"&gt;
 *   &lt;restriction base="{http://www.w3.org/2001/XMLSchema}string"&gt;
 *     &lt;enumeration value="same"/&gt;
 *     &lt;enumeration value="opposite"/&gt;
 *   &lt;/restriction&gt;
 * &lt;/simpleType&gt;
 * </pre>
 * 
 */
@XmlType(name = "e_direction")
@XmlEnum
@SuppressWarnings("all") public enum EDirection {

    @XmlEnumValue("same")
    SAME("same"),
    @XmlEnumValue("opposite")
    OPPOSITE("opposite");
    private final String value;

    EDirection(String v) {
        value = v;
    }

    public String value() {
        return value;
    }

    public static EDirection fromValue(String v) {
        for (EDirection c: EDirection.values()) {
            if (c.value.equals(v)) {
                return c;
            }
        }
        throw new IllegalArgumentException(v);
    }

}
