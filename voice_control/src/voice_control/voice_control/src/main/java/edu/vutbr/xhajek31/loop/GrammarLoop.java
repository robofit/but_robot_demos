package edu.vutbr.xhajek31.loop;

import edu.vutbr.xhajek31.*;

import edu.cmu.sphinx.frontend.util.AudioFileDataSource;
import edu.cmu.sphinx.frontend.util.Microphone;
import edu.cmu.sphinx.frontend.DataProcessingException;
import edu.cmu.sphinx.recognizer.*;
import edu.cmu.sphinx.result.Result;
import edu.cmu.sphinx.util.props.ConfigurationManager;
import static edu.cmu.sphinx.util.props.ConfigurationManagerUtils.setProperty;
import edu.cmu.sphinx.api.Configuration;
import edu.cmu.sphinx.api.SpeechResult;
import edu.cmu.sphinx.api.StreamSpeechRecognizer;
import edu.cmu.sphinx.result.WordResult;
import edu.cmu.sphinx.api.LiveSpeechRecognizer;
import edu.cmu.sphinx.frontend.util.StreamDataSource;
import edu.cmu.sphinx.linguist.language.grammar.SimpleWordListGrammar;
import edu.cmu.sphinx.jsgf.JSGFGrammar;
import javax.sound.sampled.*;

import org.ros.node.topic.Publisher;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.namespace.GraphName;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.concurrent.CancellableLoop;

/**
 * Trida starajici se o rozpoznavani s knihovnou Sphinx-4
 */

public class GrammarLoop extends CancellableLoop {

	ConfigurationManager cm;
	Recognizer recognizer;
	Kinect kinect;
	ROS ros;
	String config;
	boolean useLM = false;
	String deviceName;

	public GrammarLoop (ROS ros, boolean useLM, String deviceName) {
		this.ros = ros;
		this.useLM = useLM;
		this.deviceName = deviceName;
	}

	/**
	 * Metoda spustena pred prvni iteraci. Inicializuje veci knihovny Sphinx-4, spusti nahravani
	 */

	@Override
	protected void setup() {
		String path = RecognizerNode.class.getProtectionDomain().getCodeSource().getLocation().getPath();
		path = path.substring (0, path.lastIndexOf ('/'));
		config = path + "/../data/config/default_config.xml";
		cm = new ConfigurationManager(config);
		if (useLM) {
			setProperty(cm, "flatLinguist->grammar", "LMGrammar");
		}
		recognizer = (Recognizer) cm.lookup("recognizer");
		kinect = (Kinect)cm.lookup("kinect");
		kinect.setDeviceName (deviceName);
		recognizer.allocate ();
		try {
			kinect.startRecording();
		} catch (Exception e) {
			RecognizerNode.log.info ("Failed to start recording, check microphone");
			cancel ();
		}
	}

	/**
	 * Jeden cyklus nekonecne smycky. Ceka na rozpoznanou frazi, tu nasledne preda k publishnuti
	 * Pri chybe prerusi smycku.
	 */

	@Override
	protected void loop() throws InterruptedException {
		Result result;
		try {
			result = recognizer.recognize();
		} catch (DataProcessingException e) {
			cancel ();
			return;
		} catch (Exception e) {
			System.err.println ("Error during recognition:");
			e.printStackTrace ();
			cancel ();
			return;
		}

		if (result != null) {
			String resultText = result.getBestFinalResultNoFiller();
			System.out.println ("Said: " + resultText + " ("
			+ result.getBestPronunciationResult() + ")");
			ros.publish (resultText, 0);

		} else {
			System.out.println("Error during recognition");
			cancel ();
		}
	}

	/**
	 * Nastavi jsgf cestu a soubor.
	 * @param path absolutni cesta ke gramatice
	 * @param name jmeno souboru s gramatikou, bez .gram pripony
	 */

	public void setJSGFGrammar (String path, String name)
	{
		if (path != null) {
			setProperty(cm, "jsgfGrammar->grammarLocation", path);
		}
		setProperty(cm, "jsgfGrammar->grammarName", name);
	}

	/**
	 * Metoda, ktera bezpecne ukonci nekonecnou smycku. Nejprve ukonci nahravani, pak ukoncuje u predka.
	 */

	@Override
	public void cancel ()
	{
		if (kinect != null) {
			kinect.stopRecording ();
		}
		super.cancel ();
	}
}
