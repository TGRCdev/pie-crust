use lerp::Lerp;

/// Splits a cube of 8 corner values into
/// 8 cubes of 27 corner values
/// 
/// Before:
/// ```
///       6-----------------7
///      /                 /|
///     /                 / |
///    /                 /  |
///   /                 /   |
///  /                 /    |
/// 2-----------------3     5
/// |                 |    /
/// |                 |   /
/// |                 |  /
/// |                 | /
/// |                 |/
/// 0-----------------1
/// ```
/// 
/// After:
/// ```
///       24-------25--------26
///       /        /        /|
///      /        /        / |
///    15--------16-------17 23
///    /        /        /| /|
///   /        /        / |/ |
///  6--------7--------8  14 20
///  |        |        | /| /
///  |        |        |/ |/
///  3--------4--------5  11
///  |        |        | /
///  |        |        |/
///  0--------1--------2
///```
/// Note that the format of indices changes after
/// subdivision from binary positioning to layer-by-layer.
pub fn subdivide_values(cube: [f32; 8]) -> [[f32; 8]; 8] {
        // Construct 19 new points, for a total
        // of 27 points
        // 
        // The points are indexed from the bottom-left-back point to
        // the top-right-front point, counting in order of X, then Y,
        // then Z.
        // 
        // E.G. bottom-left-back is 0, bottom-middle-back is 1, bottom-
        // right-back is 2, middle-left-back is 3, middle-middle-back is 4,
        // etc.
        let mut points = [0.0;27];

        // First, copy the base points
        //      24-----------------26
        //      /                 /|
        //     /                 / |
        //    /                 /  |
        //   /                 /   |
        //  /                 /    |
        // 6-----------------8     20
        // |                 |    /
        // |                 |   /
        // |                 |  /
        // |                 | /
        // |                 |/
        // 0-----------------2
        // New points: 8
        // Total points: 8
        points[0] = cube[0];
        points[2] = cube[1];
        points[6] = cube[2];
        points[8] = cube[3];
        points[18] = cube[4];
        points[20] = cube[5];
        points[24] = cube[6];
        points[26] = cube[7];

        // Next, lerp between corners.
        //      24-------25--------26
        //      /                 /|
        //     /                 / |
        //   15                 17 23
        //   /                 /   |
        //  /                 /    |
        // 6--------7--------8     20
        // |                 |    /
        // |                 |   /
        // 3                 5  11
        // |                 | /
        // |                 |/
        // 0--------1--------2
        // New points: 12
        // Total points: 20
        points[1] = points[0].lerp(points[2], 0.5);
        points[3] = points[0].lerp(points[6], 0.5);
        points[5] = points[2].lerp(points[8], 0.5);
        points[7] = points[6].lerp(points[8], 0.5);
        
        points[9] = points[0].lerp(points[18], 0.5);
        points[11] = points[2].lerp(points[20], 0.5);
        points[15] = points[6].lerp(points[24], 0.5);
        points[17] = points[8].lerp(points[26], 0.5);

        points[19] = points[18].lerp(points[20], 0.5);
        points[21] = points[18].lerp(points[24], 0.5);
        points[23] = points[20].lerp(points[26], 0.5);
        points[25] = points[24].lerp(points[26], 0.5);

        // Now, lerp between midpoints.
        // We can either go from back-to-front or
        // left to right, and the values will be
        // roughly the same.
        //      24-------25--------26
        //      /        /        /|
        //     /        /        / |
        //   15--------16-------17 23
        //   /        /        /| /|
        //  /        /        / |/ |
        // 6--------7--------8  14 20
        // |        |        | /| /
        // |        |        |/ |/
        // 3--------4--------5  11
        // |        |        | /
        // |        |        |/
        // 0--------1--------2
        // New points: 6
        // Total points: 26
        points[4] = points[1].lerp(points[7], 0.5);
        points[10] = points[9].lerp(points[11], 0.5);
        points[12] = points[3].lerp(points[21], 0.5);
        points[14] = points[5].lerp(points[23], 0.5);
        points[16] = points[7].lerp(points[25], 0.5);
        points[22] = points[19].lerp(points[25], 0.5);

        // Finally, compute the midpoint.
        // (Diagram unnecessary)
        //
        // New points: 1
        // Total points: 27
        points[13] = points[4].lerp(points[22], 0.5);

        let make_cell = |start_index: usize| -> [f32; 8] {
                [
                        points[start_index  ],
                        points[start_index+1],
                        points[start_index+3],
                        points[start_index+4],
                        points[start_index+9],
                        points[start_index+10],
                        points[start_index+12],
                        points[start_index+13],
                ]
        };

        [
                make_cell(0),
                make_cell(1),
                make_cell(3),
                make_cell(4),
                make_cell(9),
                make_cell(10),
                make_cell(12),
                make_cell(13),
        ]
}