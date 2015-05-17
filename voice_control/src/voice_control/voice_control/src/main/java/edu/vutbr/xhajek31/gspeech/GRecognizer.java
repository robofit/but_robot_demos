package edu.vutbr.xhajek31.gspeech;

import edu.vutbr.xhajek31.*;

import java.util.concurrent.Callable;
import java.net.URL;
import java.net.HttpURLConnection;
import java.io.DataOutputStream;
import java.io.BufferedReader;
import java.io.InputStreamReader;

import org.json.*;

/**
 * Trida obsluhujici servery GoogleSpeech - nahraje rec na servery a pocka na odpoved, kterou nasledne publishne
 */

public class GRecognizer {

      private String url = "https://www.google.com/speech-api/v2/recognize?output=json&lang=en-us&key=";
      private ROS callbackClass;

      /**
       * @param key klic k GoogleSpeech API
       * @param cbC trida slouzici k publikovani rozpoznanych dat
       */

      public GRecognizer (String key, ROS cbC)
      {
            url += key;
            callbackClass = cbC;
      }

      /**
       * Vstupni metoda, preda rec nove vytvorenemu vlaknu
       * @param data recovy signal
       */

      public void recognize (byte[] data)
      {
            (new GRecognizerRequest (data)).run ();
      }

      /**
       * Trida obsluhujici jednotlive pozadavky, spoustena v novych vlaknech
       */

      class GRecognizerRequest implements Runnable {

            /**
             * Struktura pro predavani vysledku rozpoznavani. Obsahuje rozpoznany text a jeho pravdepodobnost.
             */

            class GResult {
                  String transcript;
                  double confidence;
            }

            byte[] data;

            public GRecognizerRequest (byte[] data)
            {
                  this.data = data;
            }

            /**
             * Samotne nahrani na server a pockani na odpoved
             */

            public void run ()
            {
                  GResult result = null;

                  try {
                        URL request = new URL(url);
                        HttpURLConnection connection = (HttpURLConnection) request.openConnection();
                        connection.setDoOutput(true);
                        connection.setDoInput(true);
                        connection.setInstanceFollowRedirects(false);
                        connection.setRequestMethod("POST");
                        connection.setRequestProperty("Content-Type", "audio/l16; rate=16000");
                        connection.setRequestProperty("User-Agent", "speech2text");
                        connection.setConnectTimeout(60000);
                        connection.setUseCaches (false);

                        DataOutputStream wr = new DataOutputStream(connection.getOutputStream ());
                        wr.write(data);
                        wr.flush();
                        wr.close();

                        BufferedReader in = new BufferedReader(new InputStreamReader(connection.getInputStream()));
                        String line;

                        while ((line = in.readLine()) != null) {
                              GResult p_result = parseResult (line);
                              System.out.println (line);
                              if (p_result != null) {
                                    if (result == null || p_result.confidence > result.confidence) {
                                          result = p_result;
                                    }
                              }
                        }
                        connection.disconnect();
                  } catch (Exception e) {
                        return;
                  }
                  if (result != null) {
                        callbackClass.publish (result.transcript, result.confidence);
                  }
            }

            /**
             * Funkce dostane odpoved z GoogleSpeech ve forme JSON a vytahne z ni rozpoznany text a
             * jeho pravdepodobnost.
             * @param json json string s vysledkem rozpoznavani
             * @return struktura s rozpoznanym textem a jeho pravdepodobnosti
             */

            GResult parseResult (String json)
            {
                  GResult result = new GResult ();

                  JSONObject obj = new JSONObject(json);
                  if (!obj.has ("result")) {
                        return null;
                  }
                  JSONArray mainResult = obj.getJSONArray("result");
                  if (mainResult.length () == 0) {
                        return null;
                  }
                  JSONArray alternatives = mainResult.getJSONObject(0).getJSONArray("alternative");
                  if (alternatives.length () > 0) {
                        JSONObject alternative = alternatives.getJSONObject(0);
                        result.transcript = alternative.getString ("transcript");
                        if (alternative.has ("confidence")) {
                              result.confidence = alternative.getDouble ("confidence");
                        } else {
                              result.confidence = -1;
                        }
                  } else {
                        return null;
                  }
                  return result;
            }
      }
}
