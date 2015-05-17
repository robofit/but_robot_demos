package edu.vutbr.xhajek31.loop;

import edu.vutbr.xhajek31.*;
import edu.vutbr.xhajek31.gspeech.*;

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
 * Trida starajici se o rozpoznavani s GoogleSpeech
 */

public class GSpeechLoop extends CancellableLoop {

	GRecognizer grec;
	VAD vad;
	ROS ros;
	String key;
	String deviceName;

	/**
	 * @param ros trida slouzici k publikovani rozpoznanych dat
	 * @param key klic k GoogleSpeech API
	 * @param deviceName jmeno zarizeni, ze ktereho se bude nahravat (napr. default, hw:0, plughw:0,1)
	 */

	public GSpeechLoop (ROS ros, String key, String deviceName) {
		super ();
		this.ros = ros;
		this.key = key;
		this.deviceName = deviceName;
	}

	/**
	 * Metoda spustena pred prvni iteraci. Inicializuje VAD knihovny Sphinx-4
	 */

	@Override
	protected void setup() {
		grec = new GRecognizer (key, ros);
		vad = new VAD (deviceName);
		if (!vad.start()) {
			RecognizerNode.log.info ("Failed to start recording, check microphone");
			cancel ();
		}
	}

	/**
	 * Jeden cyklus nekonecne smycky. Ceka na rec (ve forme byte[]), to preda k uploadnuti
	 * Pri chybe prerusi smycku.
	 */

	@Override
	protected void loop() throws InterruptedException {
		byte[] speech = vad.nextSpeech ();
		if (speech == null) {
			cancel ();
		}
		grec.recognize (speech);
	}
}
