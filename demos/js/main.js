const ephysWrapper = async function () {
  console.log(
    "%cephys\n%ca 2d rigidbody physics engine",
    "color:#f8583e;font-size:4em;font-weight:bold",
    "font-size:1.5em"
  );

  setTimeout(() => {
    $("#load-wait").animate({ opacity: "1" }, { duration: 2000 });
  }, 10000);

  console.log("loading ephys...");
  let preloadTimestamp = Date.now();
  // dynamically laod ephys.js
  // reduce page load time
  await (function () {
    return new Promise((resolve) => {
      /** @type {HTMLScriptElement} */
      let script = document.createElement("script");
      script.src = "js/ephys/ephys.js";
      script.type = "text/javascript";
      script.addEventListener("load", resolve);
      document.head.appendChild(script);
    });
  })();
  console.log(`loaded ephys.js in ${Date.now() - preloadTimestamp} ms`);
  preloadTimestamp = Date.now();
  const physics = await ephys();
  console.log(`instantiated ephys in ${Date.now() - preloadTimestamp} ms`);

  let load = $("#loading-wrapper");
  load.animate(
    { opacity: 0 },
    {
      duration: 1000,
      complete: load.remove,
    }
  );
  $("#canvas").animate({ opacity: 1 }, { duration: 1000 });

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

  const palette = ["#f8583e", "#5e9eea", "#f6c40e"];

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
        let rectWidth = body.width * Camera.size;
        let rectHeight = body.height * Camera.size;
        // save original canvas transformation
        ctx.save();
        ctx.translate(pos.x, pos.y);
        ctx.rotate(-body.rigidbody.getAngle());
        ctx.beginPath();
        ctx.rect(-rectWidth / 2, -rectHeight / 2, rectWidth, rectHeight);
        ctx.fill();
        // restore to original canvas transformation
        ctx.restore();
      }
      delete pos;
    }
  }

  let prevTimestamp = false;
  let animCallback = requestAnimationFrame(function update(timestamp) {
    if (!prevTimestamp) {
      prevTimestamp = timestamp;
    } else {
      let elapsed = timestamp - prevTimestamp;
      prevTimestamp = timestamp;

      // physics runs at twice the render rate
      let halfElapsed = elapsed / 2000; // half the elapsed time, in seconds
      world.step(halfElapsed);
      world.step(halfElapsed);
    }

    render();

    animCallback = requestAnimationFrame(update);
  });
  // for debug
  window.stopPhysics = () => {
    cancelAnimationFrame(animCallback);
  };

  return {
    ephys: physics,
    world: world,
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
      let collider = new physics.BoxCollider(new physics.Vec2(width / 2, height / 2), body);
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

  // let floor = e.box(20, 1);
  // floor.setPos(new physics.Vec2(0, -7.5));
  // floor.setStatic();

  // let body = /*e.circle(0.5)*/ e.box(1, 1);
  // body.setAngVel(-1);
  // body.setPos(new physics.Vec2(-8, 0));
  // body.setVel(new physics.Vec2(2, 0));
  // body.setAcc(gravity);

  // for (let i = 0; i < 8; i++) {
  //   let body = Math.random() < 0.5 ? e.box(1, 1) : e.circle(0.5);
  //   body.setPos(new physics.Vec2(0, 1.5 * i - 4));
  //   let v = 4 * Math.random() - 2;
  //   body.setVel(new physics.Vec2(v, 2 * Math.random() - 1));
  //   body.setAngVel(-0.5 * v);
  //   body.setAcc(gravity);
  // }

  for (let i = 0; i < 4; i++) {
    let y = 2 * (i - 1.5);

    let b1 = i < 2 ? e.box(1, 1) : e.circle(0.5);
    b1.setPos(new physics.Vec2(-2, y));
    b1.setVel(new physics.Vec2(2, 0));
    b1.setAngVel(-0.5);
    let b2 = i % 2 == 0 ? e.box(1, 1) : e.circle(0.5);
    b2.setPos(new physics.Vec2(2, y));
    // b2.setStatic();
  }
});
