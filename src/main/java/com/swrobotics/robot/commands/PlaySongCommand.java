package com.swrobotics.robot.commands;

import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to play a song using the robot's motors. Only works when the robot
 * is disabled!
 */
public final class PlaySongCommand extends Command {
    private final MusicSubsystem music;
    private final String song;

    public PlaySongCommand(MusicSubsystem music, String song) {
        this.music = music;
        this.song = song;
        addRequirements(music);
    }

    @Override
    public void initialize() {
        music.beginSong(song);
    }

    @Override
    public void end(boolean interrupted) {
        music.endSong();
    }

    @Override
    public boolean isFinished() {
        return DriverStation.isEnabled() || !music.isSongPlaying();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
