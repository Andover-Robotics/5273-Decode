package org.firstinspires.ftc.teamcode.subsystems.indexerUtil;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;

/**
 * SlotState bundles all mutable per-slot data for one indexer position.
 *
 * <p>The indexer drum has three equally-spaced slots (0, 1, 2).  Each slot
 * independently tracks:
 * <ul>
 *   <li><b>color</b> – the most recently resolved or manually assigned artifact
 *       colour (GREEN, PURPLE, EMPTY, or UNKNOWN).</li>
 *   <li><b>obs</b> – the raw sensor hit-count history used to compute a
 *       probabilistic colour decision via
 *       {@link SlotObservation#resolveWithThreshold}.</li>
 *   <li><b>wasEmpty</b> – a latch indicating whether this slot was EMPTY the
 *       last time the indexer was positioned here; used to detect the moment a
 *       new artifact enters (transition from empty → non-empty).</li>
 *   <li><b>fillingHits</b> – counts how many consecutive sensor "non-empty"
 *       readings have been seen during the current fill cycle; used by the
 *       auto-advance logic to confirm that an artifact is fully seated before
 *       advancing to the next slot.</li>
 *   <li><b>fillCycleActive</b> – latches to {@code true} once the slot
 *       transitions from stored EMPTY to sensor non-empty, so that filling
 *       confirmation counts can begin.</li>
 * </ul>
 *
 * <p>An instance of this class is held in the {@code slots[]} array inside
 * {@link Indexer}, one entry per physical slot.
 */
public class SlotState {

    // -----------------------------------------------------------------------
    // Fields
    // -----------------------------------------------------------------------

    /** The resolved (or manually assigned) colour currently stored in this slot.
     *  Defaults to UNKNOWN so the indexer knows it has not yet been classified. */
    public Indexer.ArtifactColor color = Indexer.ArtifactColor.UNKNOWN;

    /** Accumulated sensor hit counts for this slot.
     *  Used to probabilistically resolve the slot's colour from multiple
     *  sequential sensor reads rather than from a single noisy sample. */
    public SlotObservation obs = new SlotObservation();

    /** {@code true} when the slot was EMPTY the last time the colour was
     *  evaluated.  Transitions from {@code true} → {@code false} indicate that
     *  a new artifact has entered this slot, triggering fill-cycle logic. */
    public boolean wasEmpty = true;

    /** Running count of consecutive sensor-detected "non-empty" readings during
     *  the current fill cycle.  Once this reaches the required threshold the
     *  auto-advance logic accepts that the artifact is fully seated and can
     *  move the indexer to the next slot. */
    public int fillingHits = 0;

    /** {@code true} once the slot has been confirmed to be transitioning from
     *  EMPTY to a filled state (artifact detected after stored EMPTY).
     *  Reset to {@code false} when the fill cycle completes or the slot is
     *  re-initialised. */
    public boolean fillCycleActive = false;
}
