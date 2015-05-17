package edu.vutbr.xhajek31;

import edu.vutbr.xhajek31.loop.*;

import java.io.*;

import org.apache.commons.logging.Log;
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
 * Hlavni trida aplikace. Nejprve zpracuje argumenty, nasledne spusti hlavni ros node.
 */

public class RecognizerNode extends AbstractNodeMain {

      public enum RecognitionType {
            GOOGLE_SPEECH, GRAMMAR, LM
      }

      static RecognitionType type = RecognitionType.GRAMMAR;
      static boolean executeCommand = false;
      static String topicName = "voice_commands";
      static ROS ros;
      public static Log log;
      static String key = "AIzaSyCN6cKgVS3lnjDDrUQPPIYSH8rTy61twGg";
      static String deviceName = "default";
      static String jsgfPath = null;
      static String jsgfName = null;

      @Override
      public GraphName getDefaultNodeName() {
            return GraphName.of("voice_control");
      }

      /**
       * Funkce spustena rosjavou, hlavni metoda nodu.
       * Spusti vlakno, ktera ceka vstup a pripadne zastavi/povoli publikovani dat tridou ROS,
       * dale na zaklade parametru zpracovanych metodou main rozhodne, ktery loop spusti.
       */

      @Override
      public void onStart(final ConnectedNode node) {
            log = node.getLog();
            ros = new ROS (topicName, node, executeCommand);
            (new Thread () {
                  InputStreamReader reader = new InputStreamReader(System.in);

                  @Override
                  public void run ()
                  {
                        while(true) {
                              try {
                                    if ( reader.ready()) {
                                          int input = 0;
                                          input = reader.read ();
                                          if (input == 'p') {
                                                ros.togglePause ();
                                          }
                                    }
                              } catch (Exception e) {
                              }
                        }
                  }
            }).start ();
            if (type == RecognitionType.GOOGLE_SPEECH) {
                  if (key == null) {
                        System.err.println ("Recognition via GoogleSpeech API requires key set in main launch file");
                        return;
                  }
                  node.executeCancellableLoop(new GSpeechLoop (ros, key, deviceName) {});
            } else {
                  GrammarLoop loop;
                  if (type == RecognitionType.GRAMMAR) {
                        loop = new GrammarLoop (ros, false, deviceName);
                  } else {
                        loop = new GrammarLoop (ros, true, deviceName);
                  }

                  if (jsgfName != null) {
                        loop.setJSGFGrammar (jsgfPath, jsgfName);
                  }

                  node.executeCancellableLoop(loop);
            }
      }

      public static void main(String args[]) throws IOException {
            for (int i = 0; i < args.length; ++i) {
                  if (args[i].equals("-n")) {
                        if (i+1 >= args.length) {
                              System.err.println ("Wrong arguments - '-n' requires topic name after it");
                              return;
                        }
                        topicName = args[++i];
                  } else if (args[i].equals("-k")) {
                        if (i+1 >= args.length) {
                              System.err.println ("Wrong arguments - '-k' requires GSpeechApi key after it");
                              return;
                        }
                        key = args[++i];
                  } else if (args[i].equals("-d")) {
                        if (i+1 >= args.length) {
                              System.err.println ("Wrong arguments - '-d' requires device name after it");
                              return;
                        }
                        deviceName = args[++i];
                  } else if (args[i].equals("-jp")) {
                        if (i+1 >= args.length) {
                              System.err.println ("Wrong arguments - '-jp' requires path after it");
                              return;
                        }
                        jsgfPath = args[++i];
                  } else if (args[i].equals("-jf")) {
                        if (i+1 >= args.length) {
                              System.err.println ("Wrong arguments - '-jf' requires name after it");
                              return;
                        }
                        jsgfName = args[++i];
                  } else if (args[i].equals("--execute")) {
                        executeCommand = true;
                  } else if (args[i].equals("--google")) {
                        type = RecognitionType.GOOGLE_SPEECH;
                  } else if (args[i].equals("--grammar")) {
                        type = RecognitionType.GRAMMAR;
                  } else if (args[i].equals("--lm")) {
                        type = RecognitionType.LM;
                  } else if (!args[i].startsWith("__")){
                        System.err.println ("Unknown argument - '" + args[i] + "'");
                  }
            }
            String[] thisClass = { "edu.vutbr.xhajek31.RecognizerNode" };
            try {
                  org.ros.RosRun.main(thisClass);
            } catch(RosRuntimeException e) {
                  System.err.println("Interactions: ros runtime error");
            } catch(Exception e) {
                  System.err.println("Interactions: unknown error " + e);
            }
      }
}
