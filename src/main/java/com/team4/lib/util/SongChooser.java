package com.team4.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class SongChooser{
    
    public enum SongList{
        NONE,
        DEAR_MARIA_COUNT_ME_IN,
        BASKET_CASE,
        AMERICAN_IDIOT,
        HOLIDAY,
        JESUS_OF_SUBURBIA,
        SAINT_JIMMY,
        BOULEVARD_OF_BROKEN_DREAMS,
        AFRICA,
        SUMMER_OF_69,
        HOTEL_CALI,
        RICK_ROLL,
        EVA,
    }
    
    SendableChooser<SongList> mSongChooser;
    
    private SongList mCachedChoice = SongList.NONE;
    private int songChoice = (int)Double.NaN;

    public SongChooser(){
        mSongChooser = new SendableChooser<>();

        mSongChooser.setDefaultOption("None", SongList.NONE);
        mSongChooser.addOption("Dear Maria Count Me In - All Time Low", SongList.DEAR_MARIA_COUNT_ME_IN);
        mSongChooser.addOption("Basket Case - Green Day", SongList.BASKET_CASE);
        mSongChooser.addOption("American Idiot - Green Day", SongList.AMERICAN_IDIOT);
        mSongChooser.addOption("Holiday - Green Day", SongList.HOLIDAY);
        mSongChooser.addOption("Jesus Of Suburbia - Green Day", SongList.JESUS_OF_SUBURBIA);
        mSongChooser.addOption("Saint Jimmy - Green Day", SongList.SAINT_JIMMY);
        mSongChooser.addOption("Boulevard of Broken Dreams - Green Day", SongList.BOULEVARD_OF_BROKEN_DREAMS);
        mSongChooser.addOption("Africa - Toto", SongList.AFRICA);
        mSongChooser.addOption("Summer of 69' - Bryan Adams", SongList.SUMMER_OF_69);
        mSongChooser.addOption("Hotel California - The Eagles", SongList.HOTEL_CALI);
        mSongChooser.addOption("Never Gonna Give You Up - Rick Astley ", SongList.RICK_ROLL);
        mSongChooser.addOption("Cruel Angels Thesis -  Yoko Takahashi", SongList.EVA);
    }

    public void updateSelectedChoice(){
        SongList selectedSong = mSongChooser.getSelected();
        if(mCachedChoice != selectedSong){
            songChoice = getSongChoice(selectedSong);
        }

        mCachedChoice = selectedSong;
    }

    public int getSongChoice(SongList selected){
        switch(selected){
            case NONE:
                return (int) Double.NaN;
            case DEAR_MARIA_COUNT_ME_IN:
                return 0;
            case BASKET_CASE:
                return 1;
            case AMERICAN_IDIOT:
                return 2;
            case HOLIDAY:
                return 3;
            case JESUS_OF_SUBURBIA:
                return 4;
            case SAINT_JIMMY:
                return 5;
            case BOULEVARD_OF_BROKEN_DREAMS:
                return 6;
            case AFRICA:
                return 7;
            case SUMMER_OF_69:
                return 8;
            case HOTEL_CALI:
                return 9;
            case RICK_ROLL:
                return 10;
            case EVA:
                return 11;
            default:
                break;
        }
        
        return (int) Double.NaN;
    }

    public int getSelectedSong(){
        return songChoice;
    }

}