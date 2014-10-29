package edu.cmu.sphinx.frontend.util;

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
public class Kinect extends BaseDataProcessor {



    @S4Integer(defaultValue = 16000)
    public static final String PROP_SAMPLE_RATE = "sampleRate";
    @S4Boolean(defaultValue = false)
    public final static String PROP_CLOSE_BETWEEN_UTTERANCES = "closeBetweenUtterances";
    @S4Integer(defaultValue = 10)
    public final static String PROP_MSEC_PER_READ = "msecPerRead";
    @S4Integer(defaultValue = 16)
    public static final String PROP_BITS_PER_SAMPLE = "bitsPerSample";
    @S4Integer(defaultValue = 1)
    public static final String PROP_CHANNELS = "channels";
    @S4Boolean(defaultValue = false)
    public static final String PROP_BIG_ENDIAN = "bigEndian";
    @S4Boolean(defaultValue = true)
    public static final String PROP_SIGNED = "signed";
    @S4Boolean(defaultValue = false)
    public final static String PROP_KEEP_LAST_AUDIO = "keepLastAudio";
    @S4String(defaultValue = "average", range = {"average", "selectChannel"})
    public final static String PROP_STEREO_TO_MONO = "stereoToMono";
    @S4Integer(defaultValue = 0)
    public final static String PROP_SELECT_CHANNEL = "selectChannel";
    @S4String(defaultValue = "default")
    public final static String PROP_SELECT_MIXER = "selectMixer";
    @S4Integer(defaultValue = 6400)
    public final static String PROP_BUFFER_SIZE = "bufferSize";



    private final AudioFormat finalFormat = new AudioFormat (16000, 16, 1, false, false);
    private BlockingQueue<Data> audioList;
    private Utterance currentUtterance;
    private boolean doConversion;
    private volatile boolean recording;
    private volatile boolean utteranceEndReached = true;
    private RecordingThread recorder;

    // Configuration data

    private AudioFormat desiredFormat = finalFormat;
    private boolean closeBetweenUtterances;
    private boolean keepDataReference;
    private boolean signed;
    private boolean bigEndian;
    private int frameSizeInBytes = 512;
    private int msecPerRead;
    private int selectedChannel;
    private String selectedMixerIndex;
    private String stereoToMono;
    private int sampleRate;
    private int audioBufferSize;

    /**
     * @param sampleRate sample rate of the data
     * @param bitsPerSample number of bits per value.
     * @param channels number of channels.
     * @param bigEndian the endianness of the data
     * @param signed whether the data is signed.
     * @param closeBetweenUtterances whether or not the microphone will release the audio between utterances.  On
     * certain systems (Linux for one), closing and reopening the audio does not work too well. The default is false for
     * Linux systems, true for others
     * @param msecPerRead the number of milliseconds of audio data to read each time from the underlying
     * Java Sound audio device.
     * @param keepLastAudio whether to keep the audio data of an utterance around until the next utterance
     * is recorded.
     * @param stereoToMono how to convert stereo audio to mono. Currently, the possible values are
     * "average", which averages the samples from at each channel, or "selectChannel", which chooses audio only from
     * that channel. If you choose "selectChannel", you should also specify which channel to use with the
     * "selectChannel" property.
     * @param selectedChannel the channel to use if the audio is stereo
     * @param selectedMixerIndex the mixer to use.  The value can be "default," (which means let the
     * AudioSystem decide), "last," (which means select the last Mixer supported by the AudioSystem), which appears to
     * be what is often used for USB headsets, or an integer value which represents the index of the Mixer.Info that is
     * returned by AudioSystem.getMixerInfo(). To get the list of Mixer.Info objects, run the AudioTool application with
     * a command line argument of "-dumpMixers".
     */
    public Kinect(int sampleRate, int bitsPerSample, int channels,
                      boolean bigEndian, boolean signed, boolean closeBetweenUtterances, int msecPerRead, boolean keepLastAudio,
                      String stereoToMono, int selectedChannel, String selectedMixerIndex, int audioBufferSize) {
        initLogger();

        this.sampleRate = sampleRate;
        this.bigEndian = bigEndian;
        this.signed = signed;

        this.desiredFormat = new AudioFormat
                (sampleRate, bitsPerSample, channels, signed, bigEndian);

        this.closeBetweenUtterances = closeBetweenUtterances;
        this.msecPerRead = msecPerRead;
        this.keepDataReference = keepLastAudio;
        this.stereoToMono = stereoToMono;
        this.selectedChannel = selectedChannel;
        this.selectedMixerIndex = selectedMixerIndex;
        this.audioBufferSize = audioBufferSize;
    }

    public Kinect() {

    }

    /*
    * (non-Javadoc)
    *
    * @see edu.cmu.sphinx.util.props.Configurable#newProperties(edu.cmu.sphinx.util.props.PropertySheet)
    */
    @Override
    public void newProperties(PropertySheet ps) throws PropertyException {
        super.newProperties(ps);
    }


    /**
     * Constructs a Microphone with the given InputStream.
     */
    @Override
    public void initialize() {
        super.initialize();

        audioList = new LinkedBlockingQueue<Data>();

        DataLine.Info info
                = new DataLine.Info(TargetDataLine.class, desiredFormat);
/*

        if (!AudioSystem.isLineSupported(info)) {
            logger.info(desiredFormat + " not supported");
            AudioFormat nativeFormat
                    = DataUtil.getNativeAudioFormat(desiredFormat,
                    getSelectedMixer());
            if (nativeFormat == null) {
                logger.severe("couldn't find suitable target audio format");
            } else {


                doConversion = AudioSystem.isConversionSupported
                        (desiredFormat, nativeFormat);

                if (doConversion) {
                    logger.info
                            ("Converting from " + finalFormat.getSampleRate()
                                    + "Hz to " + desiredFormat.getSampleRate() + "Hz");
                } else {
                    logger.info
                            ("Using native format: Cannot convert from " +
                                    finalFormat.getSampleRate() + "Hz to " +
                                    desiredFormat.getSampleRate() + "Hz");
                }
            }
        } else {
            logger.info("Desired format: " + desiredFormat + " supported.");
       }*/
    }


    /**
     * Gets the Mixer to use.  Depends upon selectedMixerIndex being defined.
     *
     * @see #newProperties
     */
    private Mixer getSelectedMixer() {
         logger.warning ("mixer");
        if (selectedMixerIndex == null || selectedMixerIndex.equals("default")) {
            return null;
        } else {
            Mixer.Info[] mixerInfo = AudioSystem.getMixerInfo();
            if (selectedMixerIndex.equals("last")) {
                return AudioSystem.getMixer(mixerInfo[mixerInfo.length - 1]);
            } else {
                int index = Integer.parseInt(selectedMixerIndex);
                return AudioSystem.getMixer(mixerInfo[index]);
            }
        }
    }

    /**
     * Opens the audio capturing device so that it will be ready for capturing audio. Attempts to create a converter if
     * the requested audio format is not directly available.
     *
     * @return true if the audio capturing device is opened successfully; false otherwise
     *//*
    private boolean open() {
        if (audioLine != null) {
            if (!audioLine.isOpen()) {
                logger.info("open");
                try {
                    audioLine.open(finalFormat, audioBufferSize);
                } catch (LineUnavailableException e) {
                    logger.severe("Can't open microphone " + e.getMessage());
                    return false;
                }


                float sec = msecPerRead / 1000.f;
                frameSizeInBytes =
                        (finalFormat.getSampleSizeInBits() / 8) *
                                (int) (sec * finalFormat.getSampleRate()) *
                                desiredFormat.getChannels();

                logger.info("Frame size: " + frameSizeInBytes + " bytes");
            }
            return true;
        } else {
            logger.severe("Can't find microphone");
            return false;
        }
   }*/


    /**
     * Returns the format of the audio recorded by this Microphone. Note that this might be different from the
     * configured format.
     *
     * @return the current AudioFormat
     */
    public AudioFormat getAudioFormat() {
        return finalFormat;
    }


    /**
     * Returns the current Utterance.
     *
     * @return the current Utterance
     */
    public Utterance getUtterance() {
        return currentUtterance;
    }


    /**
     * Returns true if this Microphone is recording.
     *
     * @return true if this Microphone is recording, false otherwise
     */
    public boolean isRecording() {
        return recording;
    }


    /**
     * Starts recording audio. This method will return only when a START event is received, meaning that this Microphone
     * has started capturing audio.
     *
     * @return true if the recording started successfully; false otherwise
     */
    public synchronized boolean startRecording() {
        if (recording) {
            return false;
        }
        utteranceEndReached = false;
        recorder = new RecordingThread("Kinect");
        recorder.start();
        recording = true;
        return true;
    }


    /**
     * Stops recording audio. This method does not return until recording has been stopped and all data has been read
     * from the audio line.
     */
    public synchronized void stopRecording() {
        if (true) {
            if (recorder != null) {
                recorder.stopRecording();
                recorder = null;
            }
            recording = false;
        }
    }


    /**
     * This Thread records audio, and caches them in an audio buffer.
     */
    class RecordingThread extends Thread {

        private boolean done;
        private volatile boolean started;
        private long totalSamplesRead;
        private final Object lock = new Object();
        private Socket readSocket;
        private DataInputStream inStream;
        private BufferedOutputStream output = null;


        /**
         * Creates the thread with the given name
         *
         * @param name the name of the thread
         */
        public RecordingThread(String name) {
            super(name);
        }

        public InputStream getStream ()
        {
             return inStream;
        }


        /**
         * Starts the thread, and waits for recorder to be ready
         */
        @Override
        public void start() {
            started = false;
            try {
                 readSocket = new Socket("localhost", 1234);
                 inStream = new DataInputStream(readSocket.getInputStream());
            } catch (Exception e) {
                 logger.warning("could not start socket: " + e);
            }
            logger.warning ("connected");
            super.start();
            waitForStart();
        }


        /**
         * Stops the thread. This method does not return until recording has actually stopped, and all the data has been
         * read from the audio line.
         */
        public void stopRecording() {
            try {
                synchronized (lock) {
                    while (!done) {
                        lock.wait();
                    }
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            // flush can not be called here because the audio-line might has been set to  null already by the mic-thread
//    	    audioLine.flush();
        }


        /**
         * Implements the run() method of the Thread class. Records audio, and cache them in the audio buffer.
         */
        @Override
        public void run() {
            logger.warning ("thread started");
            totalSamplesRead = 0;
            logger.info("started recording");

            if (keepDataReference) {
                currentUtterance = new Utterance
                        ("Microphone", finalFormat);
            }

            audioList.add(new DataStartSignal(sampleRate));
            logger.info("DataStartSignal added");
            try {
                while (!done) {
                    Data data = readData(currentUtterance);
                    if (data == null) {
                        done = true;
                        logger.warning ("got null");
                        break;
                    }
                    audioList.add(data);
                }
            } catch (IOException ioe) {
                logger.warning("IO Exception " + ioe.getMessage());
                ioe.printStackTrace();
            }
            long duration = (long)
                    (((double) totalSamplesRead /
                            (double) 16000) * 1000.0);

            audioList.add(new DataEndSignal(duration));
            logger.info("DataEndSignal ended");
            logger.info("stopped recording");

            synchronized (lock) {
                lock.notify();
            }
        }


        /**
         * Waits for the recorder to start
         */
        private synchronized void waitForStart() {
            // note that in theory we could use a LineEvent START
            // to tell us when the microphone is ready, but we have
            // found that some javasound implementations do not always
            // issue this event when a line  is opened, so this is a
            // WORKAROUND.

            try {
                while (!started) {
                    wait();
                }
            } catch (InterruptedException ie) {
                logger.warning("wait was interrupted");
            }
        }


        /**
         * Reads one frame of audio data, and adds it to the given Utterance.
         *
         * @param utterance
         * @return an Data object containing the audio data
         * @throws java.io.IOException
         */
        private Data readData(Utterance utterance) throws IOException {

            // Read the next chunk of data from the TargetDataLine.
            byte[] data = new byte[frameSizeInBytes];

            int channels = 1;
            long firstSampleNumber = totalSamplesRead / channels;

            inStream.readFully(data);

            int numBytesRead = data.length;


            if (output == null) {
                 try {
                      output = new BufferedOutputStream(new FileOutputStream("output_32.raw", true));
                 } catch (Exception e) {}
            }
            try {
                 output.write(data);
            } catch (Exception e){
            }


            //  notify the waiters upon start
            if (!started) {
                synchronized (this) {
                    started = true;
                    notifyAll();
                }
            }
            if (numBytesRead <= 0) {
                return null;
           }
            int sampleSizeInBytes =
                    finalFormat.getSampleSizeInBits() / 8;
            totalSamplesRead += (numBytesRead / sampleSizeInBytes);

            if (numBytesRead != frameSizeInBytes) {

                if (numBytesRead % sampleSizeInBytes != 0) {
                    throw new Error("Incomplete sample read.");
                }

                data = Arrays.copyOf(data, numBytesRead);
            }

            if (keepDataReference) {
                utterance.add(data);
            }

            double[] samples;

            if (bigEndian) {
                samples = DataUtil.bytesToValues
                        (data, 0, data.length, sampleSizeInBytes, signed);
            } else {
                samples = DataUtil.littleEndianBytesToValues
                        (data, 0, data.length, sampleSizeInBytes, signed);
            }

            if (channels > 1) {
                samples = convertStereoToMono(samples, channels);
            }

            return (new DoubleData
                    (samples, (int) 16000,
                            firstSampleNumber));
        }
    }


    /**
     * Converts stereo audio to mono.
     *
     * @param samples  the audio samples, each double in the array is one sample
     * @param channels the number of channels in the stereo audio
     */
    private double[] convertStereoToMono(double[] samples, int channels) {
        assert (samples.length % channels == 0);
        double[] finalSamples = new double[samples.length / channels];
        if (stereoToMono.equals("average")) {
            for (int i = 0, j = 0; i < samples.length; j++) {
                double sum = samples[i++];
                for (int c = 1; c < channels; c++) {
                    sum += samples[i++];
                }
                finalSamples[j] = sum / channels;
            }
        } else if (stereoToMono.equals("selectChannel")) {
            for (int i = selectedChannel, j = 0; i < samples.length;
                 i += channels, j++) {
                finalSamples[j] = samples[i];
            }
        } else {
            throw new Error("Unsupported stereo to mono conversion: " +
                    stereoToMono);
        }
        return finalSamples;
    }


    /**
     * Clears all cached audio data.
     */
    public void clear() {
        audioList.clear();
    }


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

    @Override
    public Data getData() throws DataProcessingException {
         ultra_void ();
        getTimer().start();

        Data output = null;

        if (!utteranceEndReached) {
            try {
                output = audioList.take();
            } catch (InterruptedException ie) {
                throw new DataProcessingException("cannot take Data from audioList", ie);
            }
            if (output instanceof DataEndSignal) {
                utteranceEndReached = true;
            }
        }

        getTimer().stop();

        // signalCheck(output);
        return output;
    }


    /**
     * Returns true if there is more data in the Microphone.
     * This happens either if the a DataEndSignal data was not taken from the buffer,
     * or if the buffer in the Microphone is not yet empty.
     *
     * @return true if there is more data in the Microphone
     */
    public boolean hasMoreData() {
        return !(utteranceEndReached && audioList.isEmpty());
    }

    public InputStream getStream ()
    {
         return recorder.getStream();
    }
}
