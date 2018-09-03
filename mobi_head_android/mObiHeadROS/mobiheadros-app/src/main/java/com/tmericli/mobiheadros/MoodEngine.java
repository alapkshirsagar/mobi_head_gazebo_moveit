package com.tmericli.mobiheadros;

import android.content.Context;
import android.media.MediaPlayer;
import android.os.Environment;
import android.os.SystemClock;

import java.io.IOException;

/**
 * Created by tmericli on 4/15/15.
 */
public class MoodEngine implements Runnable {

    private Context mContext;

    private long mLastTime;
    private long mLastJumbleTime;

    static final long JUMBLE_EVERYTHING_DELAY = 5 * 1000;

    public MoodEngine(Context context) {
        mContext = context;

        //playMedia("moderateLaugh.m4a");
        //playMedia("smallLaugh.m4a");
    }

    public void run() {
        System.out.println("things are happening");

        final long time = SystemClock.uptimeMillis();
        final long timeDelta = time - mLastTime;
        final float timeDeltaSeconds =
                mLastTime > 0.0f ? timeDelta / 1000.0f : 0.0f;
        mLastTime = time;

        // Check to see if it's time to jumble again.
        final boolean jumble =
                (time - mLastJumbleTime > JUMBLE_EVERYTHING_DELAY);
        if (jumble) {
            mLastJumbleTime = time;

            //facialExpressionEngine.transitionToExpression(FacialExpressionEngine.Expression.SAD);
        }
    }

    public void playMedia(String fileName) {

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
