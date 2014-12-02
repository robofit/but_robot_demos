package cz.vut.fit.xhajek31;

import edu.cmu.sphinx.frontend.*;
import edu.cmu.sphinx.util.props.*;

import javax.sound.sampled.*;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.logging.Level;


import java.io.*;
import java.nio.ByteOrder;
import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;

import java.net.*;


/**
 * <p/> A Microphone captures audio data from the system's underlying audio input systems. Converts these audio data
 * into Data objects. When the method <code>startRecording()</code> is called, a new thread will be created and used to
 * capture audio, and will stop when <code>stopRecording()</code> is called. Calling <code>getData()</code> returns the
 * captured audio data as Data objects. </p> <p/> This Microphone will attempt to obtain an audio device with the format
 * specified in the configuration. If such a device with that format cannot be obtained, it will try to obtain a device
 * with an audio format that has a higher sample rate than the configured sample rate, while the other parameters of the
 * format (i.e., sample size, endianness, sign, and channel) remain the same. If, again, no such device can be obtained,
 * it flags an error, and a call <code>startRecording</code> returns false. </p>
 */
public class KinectRaw {

     private Socket readSocket;
     private DataInputStream inStream;
     private BufferedOutputStream output = null;

    public KinectRaw () {
    }

    /**
     * Starts recording audio. This method will return only when a START event is received, meaning that this Microphone
     * has started capturing audio.
     *
     * @return true if the recording started successfully; false otherwise
     */
    public void startRecording() {
            try {
                 readSocket = new Socket("localhost", 1235);
                 inStream = new DataInputStream(readSocket.getInputStream());
            } catch (Exception e) {
            }
    }


    /**
     * Stops recording audio. This method does not return until recording has been stopped and all data has been read
     * from the audio line.
     */
    public synchronized void stopRecording() {

    }


    /**
     * Clears all cached audio data.
     */


    /**
     * Reads and returns the next Data object from this Microphone, return null if there is no more audio data. All
     * audio data captured in-between <code>startRecording()</code> and <code>stopRecording()</code> is cached in an
     * Utterance object. Calling this method basically returns the next chunk of audio data cached in this Utterance.
     *
     * @return the next Data or <code>null</code> if none is available
     * @throws DataProcessingException if there is a data processing error
     */

     public void ultra_void ()
     {}

    public InputStream getStream ()
    {
         return inStream;
    }
}
