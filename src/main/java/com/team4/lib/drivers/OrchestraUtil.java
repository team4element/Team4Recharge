package com.team4.lib.drivers;

import com.ctre.phoenix.music.Orchestra;

public class OrchestraUtil {
    
    /**
     * As songs are added and removed, manage them here
     * 
     * Name convention should be: "songX.chrp"
     * Comment next to song should say song and artist
     * Would be help to specify minimum FXes to make sound good
     */
    public static String[] mSongs = new String[]{
      "song1.chrp", // Dear Maria, Count Me in: All Time Low
      "song2.chrp", // Basket Case: Green Day
      "song3.chrp",  // American Idiot: Green Day
      "song4.chrp", // Holiday: Green Day
      "song5.chrp", // Jesus Of Suburbia: Green Day
      "song6.chrp", // Saint Jimmy: Green Day
      "song7.chrp",  // Boulevard of Broken Dreams: Green Day
      "song8.chrp", // Africa: Toto
      "song9.chrp", //Summer of 69': Bryan Adams
      "song10.chrp", // Hotel California: The Eagles
      "song11.chrp", // Never Gonna Give You Up: Rick Astley
      "song12.chrp" // Cruel Angels Thesis: Yoko Takahashi

    };

    private static int mSongSelection = 1;
    private static int mTimeToPlay = 0;

    private static Orchestra mOrchestra;

    public static void addOrchestra(Orchestra orchestra){
        mOrchestra = orchestra;
    }

    public static void loadMusicSelection(boolean increase){
        if(increase){
            mSongSelection += 1;
        }else{
            mSongSelection -= 1;
        }

        if(mSongSelection >= mSongs.length){
            mSongSelection = 0;
        }
        if(mSongSelection < 0){
            mSongSelection = mSongs.length - 1;
        }

        playSong(mSongSelection);

    }
    
    public static void playSong(int songNumber){
        mOrchestra.loadMusic(mSongs[songNumber]);
        System.out.println("Changing Song to: " + mSongs[mSongSelection]);
        if(mSongSelection != songNumber){
            mSongSelection = songNumber;
        }
        mTimeToPlay = 10;
    }

    public static void playLoadedSong(){
        if (mTimeToPlay > 0) {
            --mTimeToPlay;
            if (mTimeToPlay == 0) {
                /* scheduled play request */
                
                System.out.println("Successfully started playing.");
                mOrchestra.play();
            }
        }
    }

    public static void pause(){
        if (mOrchestra.isPlaying()) {
            mOrchestra.pause();
            System.out.println("Song paused");
        }  else {
            mOrchestra.play();
            System.out.println("Playing song...");
        }
    }

    public static void stop(){
        if (mOrchestra.isPlaying()) {
            mOrchestra.stop();
            System.out.println("Song stopped.");
        }  else {
            mOrchestra.play();
            System.out.println("Playing song...");
        }
    }

    
}