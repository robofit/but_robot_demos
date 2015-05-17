package edu.vutbr.xhajek31;

import edu.cmu.sphinx.frontend.*;
import edu.cmu.sphinx.frontend.util.*;
import edu.cmu.sphinx.util.props.*;

import javax.sound.sampled.*;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.logging.Level;

/**
 * Trida starajici se o ziskavani dat z Kinectu (zvlada i jine mikrofony).
 * Koncipovana jako blok Sphinx-4 frontendu.
 * Vetsina je prevzata ze tridy edu.cmu.sphinx.frontend.util.Microphone
 */

public class Kinect extends BaseDataProcessor {

	private boolean signed = true;
	private int frameSizeInBytes = 2;
	private int sampleRate = 16000;
	private AudioFormat finalFormat = new AudioFormat( sampleRate, frameSizeInBytes * 8, 1, signed, false);
	private AudioInputStream audioStream;
	private TargetDataLine audioLine;
	private BlockingQueue<Data> audioList;
	private Utterance currentUtterance;

	private RecordingThread recorder;
	private volatile boolean recording;
	private volatile boolean utteranceEndReached = true;
	private boolean closeBetweenUtterances = false;
	private boolean keepDataReference = false;

	private String name = "plughw:2,0";

	public Kinect () {
	}

	/**
	 * Nastavi jmeno zarizeni, musi byt volano pred zacatkem nahravani (jinak se zmena neprojevi).
	 * @param name jmeno zarizeni, ze ktereho se bude nahravat (napr. default, hw:0, plughw:0,1)
	 */

	public void setDeviceName (String name) {
		this.name = name;
	}

	/**
	* Constructs a Microphone with the given InputStream.
	*/
	@Override
	public void initialize() {
		super.initialize();
		audioList = new LinkedBlockingQueue<Data>();
	}

	/**
	 * Vrati Mixer patrici zdroji se zvolenym {@link #name jmenem}
	 * @param nm jmeno zarizeni
	 */

	private Mixer getNamedTargetLine(String nm)
	{
		for (Mixer.Info mi : AudioSystem.getMixerInfo()) {
			Mixer mixer = AudioSystem.getMixer(mi);
			if (isNamedTargetDataLine(mixer, nm))
				return AudioSystem.getMixer(mi);
		}
		return null;
	}

	/**
	 * Overi, jestli danemu mixeru patri dane jmeno.
	 * Metoda volana postupne pro vsechny systemove mikrofony, dokud se nenalezne ten pozadovany
	 * @param mixer zadany mixer k overeni
	 * @param nm hledane jmeno
	 * @return true, pokud jmeno patri mixeru, jinak false
	 */

	public static boolean containsName(Mixer mixer, String nm)
	{
		Mixer.Info mi = mixer.getMixerInfo();
		String name = mi.getName().toLowerCase();
		return name.contains(nm.toLowerCase());
	}



	public static boolean isNamedTargetDataLine(Mixer mixer, String nm)
	{
		return (/* mixer.isLineSupported(tdInfo) &&*/ containsName(mixer, nm));
	}


	/**
	* Creates the audioLine if necessary and returns it.
	*/
	private TargetDataLine getAudioLine()
	{
		Mixer mixer = getNamedTargetLine (name);
		if (mixer == null) {
			return null;
		}
		System.out.println("Recording from \"" + mixer.getMixerInfo().getName() + "\"");

		TargetDataLine audioLine = null;
		DataLine.Info info = new DataLine.Info(TargetDataLine.class, finalFormat);
		try {
			audioLine = (TargetDataLine) mixer.getLine(info);
			audioLine.open(finalFormat);
		}
		catch (LineUnavailableException e) {
			System.out.println("Unable to access the capture line");
		}

		return audioLine;
	}


	private boolean open()
	{
		audioLine = getAudioLine ();
		if (audioLine == null) {
			logger.severe("Failed to open kinect device");
			System.out.println ("Could not find device by given name, choose from:");
 			for (Mixer.Info mi : AudioSystem.getMixerInfo()) {
 				Mixer mixer = AudioSystem.getMixer(mi);
 				System.out.println (mixer.getMixerInfo().getName() + "\t" + mixer.getMixerInfo().getDescription());
 			}
			return false;
		}
		audioStream = new AudioInputStream (audioLine);
		return true;
	}


	public Utterance getUtterance() {
		return currentUtterance;
	}


	public boolean isRecording() {
		return recording;
	}


	public synchronized boolean startRecording() {
		if (recording) {
			return false;
		}
		if (!open()) {
			return false;
		}
		utteranceEndReached = false;
		if (audioLine.isRunning()) {
			logger.severe("Whoops: audio line is running");
		}
		assert (recorder == null);
		recorder = new RecordingThread("Microphone");
		recorder.start();
		recording = true;
		System.out.println("Recording, can start talking...");
		return true;
	}


	/**
	* Stops recording audio. This method does not return until recording has been stopped and all data has been read
	* from the audio line.
	*/
	public synchronized void stopRecording() {
		if (audioLine != null) {
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


		/**
		* Creates the thread with the given name
		*
		* @param name the name of the thread
		*/
		public RecordingThread(String name) {
			super(name);
		}


		/**
		* Starts the thread, and waits for recorder to be ready
		*/
		@Override
		public void start() {
			started = false;
			super.start();
			waitForStart();
		}


		/**
		* Stops the thread. This method does not return until recording has actually stopped, and all the data has been
		* read from the audio line.
		*/
		public void stopRecording() {
			audioLine.stop();
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
			totalSamplesRead = 0;
			logger.info("started recording");

			if (keepDataReference) {
				currentUtterance = new Utterance
				("Kinect", audioStream.getFormat());
			}

			audioList.add(new DataStartSignal(sampleRate));
			logger.info("DataStartSignal added");
			try {
				audioLine.start();
				while (!done) {
					Data data = readData(currentUtterance);
					if (data == null) {
						done = true;
						break;
					}
					audioList.add(data);
				}
				audioLine.flush();
				if (closeBetweenUtterances) {
					/* Closing the audio stream *should* (we think)
					* also close the audio line, but it doesn't
					* appear to do this on the Mac.  In addition,
					* once the audio line is closed, re-opening it
					* on the Mac causes some issues.  The Java sound
					* spec is also kind of ambiguous about whether a
					* closed line can be re-opened.  So...we'll go
					* for the conservative route and never attempt
					* to re-open a closed line.
					*/
					audioStream.close();
					audioLine.close();
					System.err.println("set to null");
					audioLine = null;
				}
			} catch (IOException ioe) {
				logger.warning("IO Exception " + ioe.getMessage());
				ioe.printStackTrace();
			}
			long duration = (long)
			(((double) totalSamplesRead /
			(double) audioStream.getFormat().getSampleRate()) * 1000.0);

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

			long firstSampleNumber = totalSamplesRead;

			int numBytesRead = audioStream.read(data, 0, data.length);

			//  notify the waiters upon start
			if (!started) {
				synchronized (this) {
					started = true;
					notifyAll();
				}
			}

			if (logger.isLoggable(Level.FINE)) {
				logger.info("Read " + numBytesRead
				+ " bytes from audio stream.");
			}
			if (numBytesRead <= 0) {
				return null;
			}
			int sampleSizeInBytes =
			audioStream.getFormat().getSampleSizeInBits() / 8;
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


			samples = DataUtil.littleEndianBytesToValues (data, 0, data.length, sampleSizeInBytes, signed);

			return (new DoubleData
			(samples, (int) audioStream.getFormat().getSampleRate(),
			firstSampleNumber));
		}
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
	@Override
	public Data getData() throws DataProcessingException {
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
}
