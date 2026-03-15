package org.firstinspires.ftc.teamcode.subsystems.indexerUtil;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;

/**
 * SlotObservation maintains a running tally of colour classifications
 * recorded by the colour sensor while the indexer is positioned at a
 * particular slot.
 *
 * <p>Each call to {@link #record(Indexer.ArtifactColor)} increments the
 * counter for the observed colour.  After enough observations have accumulated
 * ({@code totalHits() >= MIN_HITS_FOR_DECISION} in the Indexer), the slot's
 * colour can be resolved with confidence via
 * {@link #resolveWithThreshold(double, double, double, double)}.
 *
 * <p>The {@link #trimToMax(int)} method scales all counters proportionally
 * when the total exceeds a cap, keeping the computation lightweight and
 * preventing old data from dominating indefinitely (sliding-window effect).
 */
public class SlotObservation {

    // -----------------------------------------------------------------------
    // Hit counters (one per colour category)
    // -----------------------------------------------------------------------

    /** Number of times a GREEN artifact was detected while observing this slot. */
    public int greenHits = 0;

    /** Number of times a PURPLE artifact was detected while observing this slot. */
    public int purpleHits = 0;

    /** Number of times the slot appeared EMPTY while being observed. */
    public int emptyHits = 0;

    /** Number of times the colour sensor returned UNKNOWN (artifact present but
     *  ambiguous hue – often an artifact in transit or a bad sensor read). */
    public int unknownHits = 0;

    // -----------------------------------------------------------------------
    // Reset
    // -----------------------------------------------------------------------

    /**
     * Resets all hit counters to zero.
     * Called when a slot's stored colour is re-initialised or when the indexer
     * moves away from the slot and fresh observations need to start clean.
     */
    public void reset() {
        greenHits = purpleHits = emptyHits = unknownHits = 0; // zero every counter in one assignment chain
    }

    // -----------------------------------------------------------------------
    // Recording
    // -----------------------------------------------------------------------

    /**
     * Records a single colour observation for this slot.
     * Increments the counter that corresponds to the supplied colour.
     *
     * @param c the colour classification from the colour sensor for this cycle
     */
    public void record(Indexer.ArtifactColor c) {
        switch (c) {
            case GREEN:
                greenHits++;   // saw a green artifact; accumulate green evidence
                break;
            case PURPLE:
                purpleHits++;  // saw a purple artifact; accumulate purple evidence
                break;
            case EMPTY:
                emptyHits++;   // slot appeared empty; accumulate empty evidence
                break;
            case UNKNOWN:
                unknownHits++; // ambiguous read; accumulate unknown evidence
                break;
        }
    }

    // -----------------------------------------------------------------------
    // Aggregation helpers
    // -----------------------------------------------------------------------

    /**
     * Returns the total number of observations recorded so far.
     * Used to check whether enough data has been gathered before making
     * a colour decision.
     *
     * @return sum of all four hit counters
     */
    public int totalHits() {
        return greenHits + purpleHits + emptyHits + unknownHits;
    }

    /**
     * Scales down all counters proportionally so that the total does not
     * exceed {@code maxTotal}.  This implements a simple sliding-window
     * effect: very old observations are gradually aged out while still
     * influencing the overall ratio.
     *
     * <p>If the current total is already within the limit this method is a no-op.
     *
     * @param maxTotal the maximum number of total observations to retain
     */
    public void trimToMax(int maxTotal) {
        int total = totalHits();
        if (total <= maxTotal) return; // already within the cap; nothing to do

        // Compute the scale factor needed to bring the total down to maxTotal.
        double scale = maxTotal / (double) total;

        // Round each counter to the nearest integer after scaling.
        greenHits  = (int) Math.round(greenHits  * scale);
        purpleHits = (int) Math.round(purpleHits * scale);
        emptyHits  = (int) Math.round(emptyHits  * scale);
        unknownHits= (int) Math.round(unknownHits* scale);
    }

    // -----------------------------------------------------------------------
    // Colour decision
    // -----------------------------------------------------------------------

    /**
     * Resolves the most likely slot colour from the accumulated observations
     * using per-colour fraction thresholds.
     *
     * <p>For each colour, if {@code hits / totalHits >= threshold} the method
     * returns that colour immediately (first match wins, so ordering matters:
     * GREEN is checked first, then PURPLE, EMPTY, UNKNOWN).  If no threshold
     * is exceeded the method falls back to {@link Indexer.ArtifactColor#UNKNOWN}.
     *
     * <p>Returns {@link Indexer.ArtifactColor#EMPTY} when no observations exist
     * yet (to default to "slot is free").
     *
     * @param greenThresh   fraction of total hits that must be GREEN   to declare GREEN
     * @param purpleThresh  fraction of total hits that must be PURPLE  to declare PURPLE
     * @param emptyThresh   fraction of total hits that must be EMPTY   to declare EMPTY
     * @param unknownThresh fraction of total hits that must be UNKNOWN to declare UNKNOWN
     * @return the resolved {@link Indexer.ArtifactColor}
     */
    public Indexer.ArtifactColor resolveWithThreshold(double greenThresh, double purpleThresh, double emptyThresh, double unknownThresh) {
        int total = totalHits();
        if (total == 0) return Indexer.ArtifactColor.EMPTY; // no data yet; treat as empty by default

        // Compare each colour's fraction to its threshold (first match wins).
        if (greenHits  / (double) total >= greenThresh)   return Indexer.ArtifactColor.GREEN;
        if (purpleHits / (double) total >= purpleThresh)  return Indexer.ArtifactColor.PURPLE;
        if (emptyHits  / (double) total >= emptyThresh)   return Indexer.ArtifactColor.EMPTY;
        if (unknownHits/ (double) total >= unknownThresh) return Indexer.ArtifactColor.UNKNOWN;

        return Indexer.ArtifactColor.UNKNOWN; // fallback if nothing crosses threshold
    }
}