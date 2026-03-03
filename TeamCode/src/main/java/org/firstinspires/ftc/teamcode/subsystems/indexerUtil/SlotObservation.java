package org.firstinspires.ftc.teamcode.subsystems.indexerUtil;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;

public class SlotObservation {
    public int greenHits = 0;
    public int purpleHits = 0;
    public int emptyHits = 0;
    public int unknownHits = 0;

    public void reset() { greenHits = purpleHits = emptyHits = unknownHits = 0; }

    public void record(Indexer.ArtifactColor c) {
        switch (c) {
            case GREEN:
                greenHits++;
                break;
            case PURPLE:
                purpleHits++;
                break;
            case EMPTY:
                emptyHits++;
                break;
            case UNKNOWN:
                unknownHits++;
                break;
        }
    }

    public int totalHits() { return greenHits + purpleHits + emptyHits + unknownHits; }

    public void trimToMax(int maxTotal) {
        int total = totalHits();
        if (total <= maxTotal) return;
        double scale = maxTotal / (double) total;
        greenHits = (int) Math.round(greenHits * scale);
        purpleHits = (int) Math.round(purpleHits * scale);
        emptyHits = (int) Math.round(emptyHits * scale);
        unknownHits = (int) Math.round(unknownHits * scale);
    }

    public Indexer.ArtifactColor resolveWithThreshold(double greenThresh, double purpleThresh, double emptyThresh, double unknownThresh) {
        int total = totalHits();
        if (total == 0) return Indexer.ArtifactColor.EMPTY;

        if (greenHits / (double) total >= greenThresh) return Indexer.ArtifactColor.GREEN;
        if (purpleHits / (double) total >= purpleThresh) return Indexer.ArtifactColor.PURPLE;
        if (emptyHits / (double) total >= emptyThresh) return Indexer.ArtifactColor.EMPTY;
        if (unknownHits / (double) total >= unknownThresh) return Indexer.ArtifactColor.UNKNOWN;

        return Indexer.ArtifactColor.UNKNOWN; // fallback if nothing crosses threshold
    }
}