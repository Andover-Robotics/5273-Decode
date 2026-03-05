package com.example.meepmeeptesting;
/*
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.jetbrains.annotations.NotNull;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.IOException;
import java.util.Objects;

import javax.imageio.ImageIO;

public class MeepMeepAutoPaths {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = quickBot(meepMeep,0, 0);

        RoadRunnerBotEntity myBot1 = quickBot(meepMeep,1, 1);
        //RoadRunnerBotEntity myBot1a = quickBot(meepMeep,1, 1);

        RoadRunnerBotEntity myBot2 = quickBot(meepMeep, 2, 0);

        RoadRunnerBotEntity myBot3 = quickBot(meepMeep,3, 1);
        //RoadRunnerBotEntity myBot3a = quickBot(meepMeep,3, 1);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot1)
                //.addEntity(myBot1a)
                .addEntity(myBot2)
                .addEntity(myBot3)
                //.addEntity(myBot3a)
                .start();
    }

    private static RoadRunnerBotEntity quickBot(MeepMeep meepMeep,int quadrant, int type){
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // TODO make this accurate
                .setDimensions(14.8, 17.3)
                .setColorScheme(quadrant>= 2?new ColorSchemeRedDark() : new ColorSchemeBlueDark())
                .followTrajectorySequence(TestingOpmode::createPath);

        Action action = null;
        if (type == 2) {
            action = getPark(bot, quadrant);
        } else if (quadrant % 2 == 0) {
        }
        return bot;
    }

    private static Action getPark(RoadRunnerBotEntity myBot, int quadrant) {
        int xFactor = quadrant%3==0?1:-1;
        int yFactor = quadrant>=2?-1:1;
        return null;
    }
}
*/