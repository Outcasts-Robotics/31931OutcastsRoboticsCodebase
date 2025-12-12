package org.firstinspires.ftc.teamcode.components;

public enum Tag {
    OB_PPG(23, false, new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN}),
    OB_PGP(22, false, new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE}),
    OB_GPP(21, false, new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE}),
    BlueTarget(20, true, null),
    RedTarget(24, true, null);

    public final int id;
    public final boolean isGoal;
    public final ArtifactColor[] colors;

    Tag(int id, boolean isGoal, ArtifactColor[] colors) {
        this.id = id;
        this.isGoal = isGoal;
        this.colors = colors;
    }

    public static Tag of(int id) {
        for (Tag tag : Tag.values()) {
            if (tag.id == id) {
                return tag;
            }
        }
        return null;
    }
}
