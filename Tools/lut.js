const WIDTH = 16;
const HEIGHT = 127;
const DIVISION = 16;
console.log(
    (new Array(DIVISION))
        .fill(0)
        .map((_, t) => (
            (new Array(WIDTH))
                .fill(Math.round(HEIGHT * (t + 1) / DIVISION))
                .map((H, x) => Math.round(
                    H - H * Math.sin(Math.PI * x / (2 * (WIDTH - 1)))
                ))
                .map(x => `0x${x.toString(16).padStart(2, '0')}`)
        ))
        .flat(Infinity)
        .join(', ')
)