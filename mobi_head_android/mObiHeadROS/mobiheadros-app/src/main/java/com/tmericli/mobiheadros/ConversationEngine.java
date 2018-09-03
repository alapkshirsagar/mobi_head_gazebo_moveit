package com.tmericli.mobiheadros;

import android.content.Context;
import android.media.AudioManager;
import android.speech.tts.TextToSpeech;

import java.util.Locale;

/**
 * Conversation engine handles speech understanding and generation.
 *
 * @author Tekin Mericli
 * @version 1.0 2015
 */
public class ConversationEngine {

    private TextToSpeech tts;
    private PandorabotsTalkAPI chatbot;

    public ConversationEngine(Context context) {

        // initialize the text to speech engine
        tts = new TextToSpeech(context,
                new TextToSpeech.OnInitListener() {
                    @Override
                    public void onInit(int status) {
                        if(status != TextToSpeech.ERROR){
                            tts.setLanguage(Locale.US);
                            tts.setPitch(0.5f);
                            //ttsEngine.setSpeechRate(0.75f);
                            tts.setSpeechRate(1.25f);
                        }
                    }
                });

        // initialize the chatbot
        chatbot = new PandorabotsTalkAPI();

        String request = "what are you doing?";
        String response = chatbot.askPandorabots(request);

        System.out.println("*** conversation engine: request: " + request + " response: " + response);

        // set the volume to max
        AudioManager am = (AudioManager)context.getSystemService(Context.AUDIO_SERVICE);
        int amStreamMusicMaxVol = am.getStreamMaxVolume(am.STREAM_MUSIC);
        am.setStreamVolume(am.STREAM_MUSIC, amStreamMusicMaxVol, 0);
    }

    public void speak(String text) {

        tts.speak(text, TextToSpeech.QUEUE_FLUSH, null);

    }
}
