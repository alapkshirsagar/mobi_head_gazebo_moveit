package com.tmericli.mobiheadros;

import android.media.MediaPlayer;
import android.os.Environment;

import java.io.IOException;

/**
 * The Helper class offers helper utility functions.
 * @author Tekin Mericli
 * @version 1.0 2015
 */
public class Helper {
    public static float findMax(float [] array) {
        int indexOfMax = 0;

        for (int i = 1; i < array.length; i++) {
            if (array[i] > array[indexOfMax]) {
                indexOfMax = i;
            }
        }

        return array[indexOfMax];
    }

    public static float findMin(float [] array) {
        int indexOfMin = 0;

        for (int i = 1; i < array.length; i++) {
            if (array[i] < array[indexOfMin]) {
                indexOfMin = i;
            }
        }

        return array[indexOfMin];
    }

    public static void playMedia(final String fileName) {
        System.out.println("Directory: " + Environment.getExternalStorageDirectory().getPath() + "/" + Environment.DIRECTORY_MUSIC + "/" + fileName);

        MediaPlayer mp = new MediaPlayer();
        try {
            mp.setDataSource(Environment.getExternalStorageDirectory().getPath() + "/" + Environment.DIRECTORY_MUSIC + "/" + fileName);
            mp.prepare();
            int duration = mp.getDuration();
            mp.start();
            Thread.sleep(duration + 100);
        } catch (IOException ex) {
            ex.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
