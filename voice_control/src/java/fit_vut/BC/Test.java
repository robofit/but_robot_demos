package fit_vut.BC;

import org.openkinect.freenect.*;
import org.openkinect.freenect.util.*;

import java.nio.*;
import java.util.ArrayList;
import java.util.List;
import java.io.*;

import com.sun.jna.*;
import com.sun.jna.ptr.PointerByReference;

public class Test {

     static Context ctx;
     static Device dev;

     public static void initKinect() {
        ctx = Freenect.createContext();
        ctx.setLogHandler(new Jdk14LogHandler());
        ctx.setLogLevel(LogLevel.SPEW);
        if (ctx.numDevices() > 0) {
            dev = ctx.openDevice(0);
        } else {
            System.err.println("WARNING: No kinects detected, hardware tests will be implicitly passed.");
        }
     }

     public static void shutdownKinect() {
          if (ctx != null) {
               if (dev != null) {
                    dev.close();
               }
               ctx.shutdown();
          }
     }

     public static void testAudio() throws InterruptedException {

        final Object lock = new Object();
        final long start = System.nanoTime();
        dev.startAudio(new AudioHandler() {
            int frameCount = 0;
            BufferedOutputStream output = null;

            @Override
            public void onFrameReceived(Pointer dev, int num_samples, Pointer mic) {
                /*frameCount++;
                if (frameCount == 500) {
                    synchronized (lock) {
                        lock.notify();
                        System.out.format("Got %d video frames in %4.2fs%n", frameCount,
                                (((double) System.nanoTime() - start) / 1000000000));
                         try {
                              output.close();
                         } catch (Exception e){
                         }
                    }
                }
                byte[] byteArray = mic1.getByteArray (0, num_samples);
                IntBuffer intBuf = ByteBuffer.wrap(byteArray).order(ByteOrder.LITTLE_ENDIAN).asIntBuffer();
                int[] array = new int[intBuf.remaining()];
                intBuf.get(array);
                if (output == null) {
                     try {
                          output = new BufferedOutputStream(new FileOutputStream("output_32.raw", true));
                     } catch (Exception e) {}
                }
                try {
                     output.write(byteArray);
                } catch (Exception e){
                }*/
                //test.write(byteArray, "output_32.raw");
            }
        });
        synchronized (lock) {
             lock.wait(2000);
        }
   }


}
