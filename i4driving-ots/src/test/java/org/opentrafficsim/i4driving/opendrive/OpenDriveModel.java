package org.opentrafficsim.i4driving.opendrive;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;

import javax.xml.bind.JAXBException;
import javax.xml.parsers.ParserConfigurationException;

import org.opentrafficsim.animation.DefaultAnimationFactory;
import org.opentrafficsim.animation.gtu.colorer.DefaultSwitchableGtuColorer;
import org.opentrafficsim.animation.gtu.colorer.GtuColorer;
import org.opentrafficsim.core.dsol.AbstractOtsModel;
import org.opentrafficsim.core.dsol.OtsAnimator;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.geometry.OtsGeometryException;
import org.opentrafficsim.core.network.Network;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.draw.OtsDrawingException;
import org.opentrafficsim.road.network.RoadNetwork;
import org.xml.sax.SAXException;

import nl.tudelft.simulation.dsol.SimRuntimeException;

/**
 * Test model based on OpenDRIVE file.
 */
public class OpenDriveModel extends AbstractOtsModel
{

    /** */
    private static final long serialVersionUID = 1L;

    /** Colorer. */
    static final GtuColorer COLORER = new DefaultSwitchableGtuColorer();

    /** File. */
    private final String file;

    /** Network. */
    private RoadNetwork network;

    /**
     * Constructor.
     * @param simulator simulator
     * @param file file
     */
    public OpenDriveModel(final OtsSimulatorInterface simulator, final String file)
    {
        super(simulator);
        this.file = file;
    }

    @Override
    public Network getNetwork()
    {
        return this.network;
    }

    @Override
    public void constructModel() throws SimRuntimeException
    {
        try
        {
            InputStream stream;
            if (this.file.startsWith("/opendrive/"))
            {
                stream = OpenDriveParserTest.class.getResource(this.file).openStream();
            }
            else
            {
                stream = new FileInputStream(this.file);
            }
            OpenDriveParser parser = OpenDriveParser.parseStream(stream);
            this.network = new RoadNetwork("roadNetwork", getSimulator());
            parser.build(this.network);
            OpenDriveParser.buildConflicts(this.network);
            if (getSimulator() instanceof OtsAnimator)
            {
                DefaultAnimationFactory.animateXmlNetwork(this.network, COLORER);
            }
        }
        catch (IOException | JAXBException | SAXException | ParserConfigurationException | NetworkException
                | OtsGeometryException | OtsDrawingException ex)
        {
            throw new SimRuntimeException(ex);
        }
    }

}
