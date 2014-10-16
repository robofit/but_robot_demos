/*
 * Copyright 1999-2013 Carnegie Mellon University.
 * Portions Copyright 2004 Sun Microsystems, Inc.
 * Portions Copyright 2004 Mitsubishi Electric Research Laboratories.
 * All Rights Reserved.  Use is subject to license terms.
 *
 * See the file "license.terms" for information on usage and
 * redistribution of this file, and for a DISCLAIMER OF ALL
 * WARRANTIES.
 */

package fit_vut.BC;

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


/**
 * A simple example that shows how to transcribe a continuous audio file that
 * has multiple utterances in it.
 */
public class BC {

    public static void main(String[] args) throws Exception {
          ConfigurationManager cm;
          KinectRaw kinect;
          Recognizer recognizer;
          StreamDataSource source;

          cm = new ConfigurationManager("./data/config/default_config.xml");
          recognizer = (Recognizer) cm.lookup("recognizer");
          kinect = new KinectRaw ();
          source = (StreamDataSource)cm.lookup("dataSource");
          recognizer.allocate ();
          kinect.startRecording();
          //AudioSystem.write(new AudioInputStream(kinect.getStream(), new AudioFormat (16000, 16, 1, true, false), 80000), AudioFileFormat.Type.WAVE, new File ("./tmp.wav"));
          source.setInputStream(kinect.getStream());
          System.out.println("Say something");
          while (true) {
               Result result = recognizer.recognize();

               if (result != null) {
                    String resultText = result.getBestFinalResultNoFiller();
                    System.out.println ("Said: " + resultText + " ("
                                         + result.getBestPronunciationResult() + ")");
               } else {
                    System.out.println("Try again.");
               }
          }
     }
}
