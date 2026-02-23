package org.firstinspires.ftc.teamcode.subsystems.indexerUtil;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;

public class SlotState {
    public Indexer.ArtifactColor color = Indexer.ArtifactColor.UNKNOWN;
    public SlotObservation obs = new SlotObservation();
    public boolean wasEmpty = true;
    public int fillingHits = 0;          // counts sensorNonEmpty hits during a fill cycle
    public boolean fillCycleActive = false;  // latched after stored EMPTY + sensor nonempty
}
