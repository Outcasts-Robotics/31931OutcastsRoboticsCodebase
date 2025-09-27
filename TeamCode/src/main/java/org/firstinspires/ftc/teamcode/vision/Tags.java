package org.firstinspires.ftc.teamcode.vision;

public enum Tags {
    BlueTarget(20, "BlueTarget"),
    RedTarget(24, "RedTarget"),
    Obelisk_GPP(21, "Obelisk_GPP"),
    Obelisk_PGP(22, "Obelisk_PGP"),
    Obelisk_PPG(23, "Obelisk_PPG");

    final int id;
    final String name;

    Tags(int id, String name) {
        this.id = id;
        this.name = name;
    }

    public static boolean isPositionTagId(int tagId) {
        return tagId == BlueTarget.id || tagId == RedTarget.id;
    }
}
