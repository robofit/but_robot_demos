package edu.vutbr.xhajek31.gspeech;

import edu.vutbr.xhajek31.*;

import edu.cmu.sphinx.frontend.util.*;
import edu.cmu.sphinx.util.props.ConfigurationManager;
import edu.cmu.sphinx.frontend.util.StreamDataSource;
import edu.cmu.sphinx.util.*;
import edu.cmu.sphinx.frontend.*;
import edu.cmu.sphinx.frontend.endpoint.SpeechEndSignal;
import edu.cmu.sphinx.frontend.endpoint.SpeechStartSignal;
import edu.cmu.sphinx.util.props.*;

import java.util.List;
import java.util.ArrayList;
import javax.sound.sampled.AudioFileFormat;
import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import java.io.*;

/**
 * Trida implementujici voice activity detector, data ziskava z frontendu Sphinx-4.
 * Pouziva nastaveni knihovny ze souboru frontend_config.xml.
 */

public class VAD {

	ConfigurationManager cm;
	FrontEnd frontend;
	Kinect kinect;
	String config;
	String deviceName;

	/**
	 * Nacte konfiguraci knihovny Sphinx-4, vyhleda Kinect (coby mikrofon) a frontend (odkud bude nacitat data),
	 * ktery pak necha inicializovat
	 * @param deviceName jmeno zarizeni, ze ktereho se bude nahravat (napr. default, hw:0, plughw:0,1)
	 */

	public VAD (String deviceName)
	{
		this.deviceName = deviceName;
		String path = RecognizerNode.class.getProtectionDomain().getCodeSource().getLocation().getPath();
		path = path.substring (0, path.lastIndexOf ('/'));
		config = path + "/../data/config/frontend_config.xml";
		cm = new ConfigurationManager(config);
		frontend = (FrontEnd) cm.lookup("endpointer");
		kinect = (Kinect) cm.lookup ("kinect");
		kinect.setDeviceName (deviceName);
		frontend.initialize();
	}

	/**
	 * Metoda spoustici nahravani z mikrofonu,
	 * musi byt zavolana pred prvnim volanim {@link #nextSpeech() nextSpeech}
	 */

	public boolean start ()
	{
		try {
			if (!kinect.startRecording()) {
 				RecognizerNode.log.error ("Failed to start recording, check microphone");
 				cancel ();
 			}
		} catch (Exception e) {
			return false;
		}
		return true;
	}

	/**
	 * Blokujici metoda vracejici recovy signal, ceka na data z knihovny Sphinx-4
	 * @return pole bajtu (little-endian) s recovym signalem
	 */

	public byte[] nextSpeech ()
	{
		int size = 0;
		boolean inSpeech = false;

		ByteArrayOutputStream baos = null;
		DataOutputStream dos = null;

		while (true) {
			Data data;
			try {
				data = frontend.getData();
			} catch (DataProcessingException e) {
				kinect.stopRecording ();
				return null;
			}

			if (data == null) {
				return null;
			}

			if ((data instanceof DataEndSignal) || (data instanceof SpeechEndSignal)) {
				RecognizerNode.log.debug ("[VAD] detected end of speech, starting recognition");
				return toLittleEndian(baos.toByteArray());
			}



			if (data instanceof SpeechStartSignal) {
				inSpeech = true;
				baos = new ByteArrayOutputStream();
				dos = new DataOutputStream(baos);
				RecognizerNode.log.debug ("[VAD] detected start of speech");
			}

			if ((data instanceof DoubleData || data instanceof FloatData) && inSpeech) {
				DoubleData dd = data instanceof DoubleData ? (DoubleData) data : DataUtil.FloatData2DoubleData((FloatData) data);
				double[] values = dd.getValues();

				for (double value : values) {
					try {
						dos.writeShort(new Short((short) value));
					} catch (IOException e) {
						e.printStackTrace();
					}
				}
			}
		}
	}

	/**
	 * Prevede big-endian na little-endian (pro 16bitove vzorky).
	 * Pouziva algoritmus prohozeni i-teho a i+1-teho prvku.
	 * @param data pole k prevedeni
	 * @return LE vysledek konverze
	 */

	public byte[] toLittleEndian (byte [] data)
	{
		if (data == null) {
			return null;
		}
		for (int i = 0; i < data.length - 1; i += 2) {
			byte b = data [i];
			data [i] = data[i+1];
			data[i+1] = b;
		}
		return data;
	}

}
