// Client
// -----------------

let state = {
    x: $('.x'),
    y: $('.y'),
    sliders: {
        linear: 0.25,
        angular: 0.5
    },
    velocities: {
        linear: 0.0,
        angular: 0.0
    },
    mapData: null,
    cleanMap: null,
    canvas: null
}

function setSliders() {

    let linearSlider = document.getElementById("linearRange");
    let angularSlider = document.getElementById("angularRange");

    let linearVel = $(".lin");
    let angularVel = $(".ang");

    linearSlider.oninput = function () {
        state.sliders.linear = (linearSlider.value / 200);
        linearVel.text("Absolute Linear Vel : " + (state.sliders.linear).toString().substring(0, 4));
    }

    angularSlider.oninput = function () {
        state.sliders.angular = (angularSlider.value / 100);
        angularVel.text("Absolute Angular Vel: " + (state.sliders.angular).toString().substring(0, 4));
    }
}

function assignEvents() {
    $(".coord").on("click", function () {
        $.ajax({
            "url": "/landmark",
            "method": "POST",
            "timeout": 0,
        }).done();
    })

    $(".forward").on("click", function () {
        if (state.velocities.linear < 0.0)
            state.velocities.linear = 0.0;
        else
            state.velocities.linear = Math.abs(state.sliders.linear);
    });

    $(".backwards").on("click", function () {
        if (state.velocities.linear > 0.0)
            state.velocities.linear = 0.0;
        else
            state.velocities.linear = -Math.abs(state.sliders.linear);
    });

    $(".stop").on("click", function () {
        state.velocities.linear = 0.0;
        state.velocities.angular = 0.0;
    });

    $(".left").on("click", function () {
        if (state.velocities.angular < 0.0)
            state.velocities.angular = 0.0;
        else
            state.velocities.angular = Math.abs(state.sliders.angular);
    });

    $(".right").on("click", function () {
        if (state.velocities.angular > 0.0)
            state.velocities.angular = 0.0;
        else
            state.velocities.angular = -Math.abs(state.sliders.angular);
    });
}

function getMap() {
    $.ajax({
        "url": "/map",
        "method": "GET",
        "timeout": 0,
    }).done(response => {
        state.mapData = response;
        state.canvas = document.getElementById("map_canvas");
        let ctx = state.canvas.getContext("2d");

        ctx.canvas.width = state.mapData.width;
        ctx.canvas.height = state.mapData.height;

        let imageData = ctx.getImageData(0, 0, ctx.canvas.width, ctx.canvas.height);
        let data = imageData.data;

        let idx = 0;

        /*
        ctx.canvas.width = state.mapData.height;
        ctx.canvas.height = state.mapData.width;

        //rotate 90 degrees and start from 0,0

        for (let j = ctx.canvas.width - 1; j >= 0; j--) {
            for (let i = ctx.canvas.height - 1; i >= 0; i--) {*/

        // start from 0,0
        for (let i = ctx.canvas.height - 1; i >= 0; i--) {
            for (let j = 0; j < ctx.canvas.width; j++) {

                let pixel = response.data[idx];
                let pIdx = 4 * ((i * ctx.canvas.width) + j);

                switch (pixel) {
                    case -1 :
                        data[pIdx] = 128;
                        data[pIdx + 1] = 128;
                        data[pIdx + 2] = 128;
                        data[pIdx + 3] = 255;
                        break;
                    case 0:
                        data[pIdx] = 255;
                        data[pIdx + 1] = 255;
                        data[pIdx + 2] = 255;
                        data[pIdx + 3] = 255;
                        break;
                    case 100:
                        data[pIdx] = 0;
                        data[pIdx + 1] = 0;
                        data[pIdx + 2] = 0;
                        data[pIdx + 3] = 255;
                        break;
                    default:
                        console.log("pixel", pixel);
                }

                idx++;
            }
        }

        ctx.putImageData(imageData, 0, 0);
        state.cleanMap = imageData;
    });
}


function publishVel() {
    // if a robot has a timeout it will stop if no packets are sent
    if (state.velocities.linear !== 0.0 || state.velocities.angular !== 0.0)
        $.ajax({
            "url": "/move",
            "data": state.velocities,
            "method": "POST",
            "timeout": 0,
        }).done();

}

async function setPosition() {
    $.ajax({
        "url": "/location",
        "method": "GET",
        "timeout": 0,
    }).done(response => {
        state.x.text("x: " + response.x);
        state.y.text("y: " + response.y);
    });

    $.ajax({
        "url": "/amcl",
        "method": "GET",
        "timeout": 0,
    }).done(response => {


        let _origin = {
            x: Math.abs(state.mapData.origin.position.x / state.mapData.resolution),
            y: Math.abs(state.mapData.origin.position.y / state.mapData.resolution)
        };

        console.log(_origin);

        let _x = ((response.x / state.mapData.resolution) + _origin.x) | 0; // bitwise to integer
        let _y = ((-response.y / state.mapData.resolution) + _origin.y) | 0; // flip y

        console.log("x", _x, "y", _y);

        let ctx = state.canvas.getContext("2d");

        ctx.beginPath();
        ctx.rect(_x, _y, 5, 5);
        ctx.fillStyle = "lime";
        ctx.fill();

    });
}

$(document).ready(function () {
    setSliders();
    assignEvents();
    getMap();
    setInterval(publishVel, 100); //10hz
    setInterval(setPosition, 1000); //1hz
});
