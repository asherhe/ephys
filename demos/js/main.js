const ephysWrapper = async function () {
  let preloadTimestamp = Date.now();
  var physics;
  do {
    physics = await ephys();
  } while (!physics);
  console.log(`loaded ephys in ${Date.now() - preloadTimestamp} ms`);

  let load = $("#loading-wrapper");
  load.animate(
    { opacity: "0" },
    {
      duration: 1000,
      complete: load.remove,
    }
  );

  const TAU = Math.PI * 2;

  /** @type {HTMLCanvasElement} */
  let canvas = document.getElementById("canvas"),
    ctx = canvas.getContext("2d");

  let canvasToWindowSize = () => {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
  };
  window.addEventListener("resize", canvasToWindowSize);
  canvasToWindowSize();

  let Camera = {
    x: 0,
    y: 0,
    size: 40, // pixels per unit
    toScreenCoords: function (x, y) {
      return {
        x: (x - Camera.x) * Camera.size + canvas.width / 2,
        y: (Camera.y - y) * Camera.size + canvas.height / 2,
      };
    },
    toWorldCoords: function (x, y) {
      return {
        x: (x - canvas.width / 2) / Camera.size + Camera.x,
        y: -((y - canvas.height / 2) / Camera.size + Camera.y),
      };
    },
  };

  let world = new physics.World();
  /**
   * @typedef {{type: "circle"|"box", radius?: number, width?: number, height?: number rigidbody: ep.Rigidbody}} Shape
   * @type {Shape[]}
   */
  let bodies = [];

  const palette = ["#f8583e", "#5e9eea", "#e3b021"];

  function render() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    let i = 0;
    for (let body of bodies) {
      ctx.fillStyle = palette[i++];
      if (i >= palette.length) i = 0;

      let pos = body.rigidbody.getPos();
      pos = Camera.toScreenCoords(pos.x, pos.y);
      if (body.type == "circle") {
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, body.radius * Camera.size, 0, TAU);
        ctx.fill();
      } else {
        ctx.fillRect(
          pos.x - (body.width * Camera.size) / 2,
          pos.y - (body.height * Camera.size) / 2,
          body.width * Camera.size,
          body.height * Camera.size
        );
      }
      delete pos;
    }
  }

  let prevTimestamp = false;
  let update = function (timestamp) {
    if (!prevTimestamp) {
      prevTimestamp = timestamp;
    } else {
      let elapsed = timestamp - prevTimestamp;
      prevTimestamp = timestamp;

      // physics runs at twice the render rate
      // let halfElapsed = elapsed / 2000; // half the elapsed time, in seconds
      // world.step(halfElapsed);
      // world.step(halfElapsed);
      world.step(elapsed / 1000);
    }

    render();

    requestAnimationFrame(update);
  };
  requestAnimationFrame(update);

  return {
    ephys: physics,
    circle: function (radius) {
      let body = new physics.Rigidbody();
      let collider = new physics.CircleCollider(radius, body);
      body.setCollider(collider);
      world.addBody(body, collider);
      bodies.push({
        type: "circle",
        radius: radius,
        rigidbody: body,
      });
      return body;
    },
    box: function (width, height) {
      let body = new physics.Rigidbody();
      let collider = new physics.BoxCollider(new physics.Vec2(width, height), body);
      body.setCollider(collider);
      world.addBody(body, collider);
      bodies.push({
        type: "box",
        width: width,
        height: height,
        rigidbody: body,
      });
      return body;
    },
  };
};

ephysWrapper().then((e) => {
  const physics = e.ephys;
  let gravity = new physics.Vec2(0, -5);

  let circles = [];
  for (let i = 0; i < 3; i++) {
    let circle = e.circle(0.5);
    circle.setVel(new physics.Vec2(Math.random() * 6 - 3, Math.random() * 5));
    circle.setAcc(gravity);
    circles.push(circle);
  }

  let floor = e.box(8, 1);
  floor.setPos(new physics.Vec2(0, -2));
});
