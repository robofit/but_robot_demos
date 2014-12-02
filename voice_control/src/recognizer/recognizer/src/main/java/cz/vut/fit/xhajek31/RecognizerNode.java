package cz.vut.fit.xhajek31;

import java.io.*;


import edu.cmu.sphinx.frontend.util.AudioFileDataSource;
import edu.cmu.sphinx.frontend.util.Microphone;
import edu.cmu.sphinx.frontend.util.Kinect;
import edu.cmu.sphinx.recognizer.*;
import edu.cmu.sphinx.result.Result;
import edu.cmu.sphinx.util.props.ConfigurationManager;
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
 * A simple example that shows how to transcribe a continuous audio file that
 * has multiple utterances in it.
 */
public class RecognizerNode extends AbstractNodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("voice_control");
  }

  @Override
  public void onStart(final ConnectedNode node) {
	node.executeCancellableLoop(new CancellableLoop() {

		ConfigurationManager cm;
          	KinectRaw kinect;
          	Recognizer recognizer;
          	StreamDataSource source;
	  	ROS ros;
		String config;

		@Override
      		protected void setup() {
			String path = RecognizerNode.class.getProtectionDomain().getCodeSource().getLocation().getPath();
			path = path.substring (0, path.lastIndexOf ('/'));
			config = path + "/../data/config/default_config.xml";
        		ros = new ROS ("voice_commands", node);
			cm = new ConfigurationManager(config);
          		recognizer = (Recognizer) cm.lookup("recognizer");
          		kinect = new KinectRaw ();
          		source = (StreamDataSource)cm.lookup("dataSource");
          		recognizer.allocate ();
          		kinect.startRecording();
          		source.setInputStream(kinect.getStream());
			ros.send (0, "starting recognizing");
      		}
          	
          	@Override
      		protected void loop() throws InterruptedException {
	               Result result = recognizer.recognize();
	
               		if (result != null) {
                    		String resultText = result.getBestFinalResultNoFiller();
                    		System.out.println ("Said: " + resultText + " ("
                                         + result.getBestPronunciationResult() + ")");
                    		ros.send (0, resultText);
			
               		} else {
                    		System.out.println("Error during recognition");
                    		cancel ();
               		}
          	}
	});
  }

 public static void main(String args[]) throws java.io.IOException {
        String[] thisClass = { "cz.vut.fit.xhajek31.RecognizerNode" };
          try {
              org.ros.RosRun.main(thisClass);
          } catch(RosRuntimeException e) {
              System.out.println("Interactions: ros runtime error");
          } catch(Exception e) {
              System.out.println("Interactions: unknown error " + e);
          }
    }
}

