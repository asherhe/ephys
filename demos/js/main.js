const ephysWrapper = async function () {
  let preloadTimestamp = Date.now();
  const ep = await ephys();
  console.log(`loaded ephys in ${Date.now() - preloadTimestamp} ms`);

  const config = {
    particleSize: 8,
    hoverParticleSize: 10,
    TAU: 2 * Math.PI,
  };

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
    size: 80, // pixels per unit
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

  let pWorld = new ep.ParticleWorld();
  let particles = [],
    pfgens = [],
    pcgens = [],
    springs = [],
    links = [];

  let hoverParticle,
    grabParticle,
    mousePos = { x: 0, y: 0 },
    mouseWorldPos = new ep.Vec2(),
    grabSpring = new ep.ParticleSpring(mouseWorldPos, 2, 0);
  canvas.addEventListener("contextmenu", (e) => {
    e.preventDefault();
  });
  let inputdown = function () {
      if (hoverParticle) {
        grabParticle = hoverParticle;
        grabSpring.k = grabParticle.getMass() * 5;
        pWorld.addPFGen(grabParticle, grabSpring);
        canvas.style.cursor = "grabbing";
      }
    },
    inputup = function () {
      if (grabParticle) {
        pWorld.removePFGen(grabParticle, grabSpring);
        grabParticle = false;
        canvas.style.cursor = "initial";
      }
    },
    inputmove = function (x, y) {
      mousePos.x = x;
      mousePos.y = y;
      pos = Camera.toWorldCoords(mousePos.x, mousePos.y);
      mouseWorldPos.set(pos.x, pos.y);
    };
  canvas.addEventListener("mousedown", (e) => {
    if (e.button == 0) inputdown();
  });
  // canvas.addEventListener("touchstart", (e) => {
  //   e.preventDefault();
  //   let touch = e.touches[0];
  //   inputmove(touch.clientX - canvas.clientLeft, touch.clientY - canvas.clientTop);
  //   inputdown();
  // });
  canvas.addEventListener("mouseup", (e) => {
    if (e.button == 0) inputup();
  });
  canvas.addEventListener("touchend", inputup);
  canvas.addEventListener("mousemove", (e) => {
    inputmove(e.offsetX, e.offsetY);
  });
  canvas.addEventListener("touchmove", (e) => {
    e.preventDefault();
    let touch = e.touches[0];
    inputmove(touch.clientX - canvas.clientLeft, touch.clientY - canvas.clientTop);
    inputdown();
  });

  function render() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    ctx.strokeStyle = "#555";
    ctx.lineCap = "round";
    ctx.lineJoin = "round";

    // draw springs
    ctx.lineWidth = 2;
    for (let spring of springs) {
      let s = spring[0],
        p = spring[1];
      let pos = s.getEnd().getPos();
      pos = Camera.toScreenCoords(pos.x, pos.y);
      ctx.beginPath();
      ctx.moveTo(pos.x, pos.y);
      pos = p.getPos();
      pos = Camera.toScreenCoords(pos.x, pos.y);
      ctx.lineTo(pos.x, pos.y);
      ctx.stroke();
    }

    // draw links
    ctx.lineWidth = 4;
    for (let link of links) {
      let p1 = link.getParticle(0),
        p2 = link.getParticle(1);
      let pos = p1.getPos();
      pos = Camera.toScreenCoords(pos.x, pos.y);
      ctx.beginPath();
      ctx.moveTo(pos.x, pos.y);
      pos = p2.getPos();
      pos = Camera.toScreenCoords(pos.x, pos.y);
      ctx.lineTo(pos.x, pos.y);
      ctx.stroke();
    }

    ctx.fillStyle = "#f8583e";
    for (let p of particles) {
      let pos = p.getPos();
      pos = Camera.toScreenCoords(pos.x, pos.y);
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, p == hoverParticle ? config.hoverParticleSize : config.particleSize, 0, config.TAU);
      ctx.fill();
    }

    if (grabParticle) {
      let pos = grabParticle.getPos();
      pos = Camera.toScreenCoords(pos.x, pos.y);
      ctx.strokeStyle = "#50e22d";
      ctx.lineWidth = 2;
      // arrow(pos.x, pos.y, mousePos.x, mousePos.y, 2);
      ctx.beginPath();
      ctx.moveTo(pos.x, pos.y);
      ctx.lineTo(mousePos.x, mousePos.y);
      ctx.stroke();
    }
  }

  function getHoverParticle() {
    let size2 = config.particleSize * config.particleSize;
    hoverParticle = false;
    for (let p of particles) {
      let pos = p.getPos();
      pos = Camera.toScreenCoords(pos.x, pos.y);
      let dx = pos.x - mousePos.x,
        dy = pos.y - mousePos.y;
      if (dx * dx + dy * dy <= size2) {
        hoverParticle = p;
        break;
      }
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
      let halfElapsed = elapsed / 2000; // half the elapsed time, in seconds
      pWorld.step(halfElapsed);
      pWorld.step(halfElapsed);
    }

    getHoverParticle();
    render();

    requestAnimationFrame(update);
  };
  requestAnimationFrame(update);

  return {
    ephys: ep,
    addParticle: function (particle) {
      pWorld.addParticle(particle);
      particles.push(particle);
    },
    removeParticle: function (particle) {
      pWorld.removeParticle(particle);
      delete particles[particles.findIndex(particle)];
    },
    addPFGen: function (particle, pfgen) {
      pWorld.addPFGen(particle, pfgen);
      pfgens.push([pfgen, particle]);
    },
    removePFGen: function (particle, pfgen) {
      pWorld.removePFGen(particle, pfgen);
      delete pfgens[pfgens.findIndex([pfgen, particle])];
    },
    addSpring: function (particle, spring) {
      pWorld.addPFGen(particle, spring);
      pfgens.push([spring, particle]);
      springs.push([spring, particle]);
    },
    removeSpring: function (particle, spring) {
      pWorld.removePFGen(particle, spring);
      delete pfgens[pfgens.findIndex([spring, particle])];
      delete springs[springs.findIndex([spring, particle])];
    },
    addPCGen: function (pcgen) {
      pWorld.addPContactGenerator(pcgen);
      pcgens.push(pcgen);
    },
    removePCGen: function (pcgen) {
      pWorld.removePContactGenerator(pcgen);
      delete pcgens[pcgens.findIndex(pcgen)];
    },
    addLink: function (link) {
      pWorld.addPContactGenerator(link);
      pcgens.push(link);
      links.push(link);
    },
    removeLink: function (link) {
      pWorld.removePContactGenerator(link);
      delete pcgens[pcgens.findIndex(link)];
      delete links[links.findIndex(link)];
    },
  };
};

ephysWrapper().then((e) => {
  let p1 = new e.ephys.Particle();
  p1.setPos(new e.ephys.Vec2(1, 0));
  e.addParticle(p1);
  let p2 = new e.ephys.Particle();
  p2.setPos(new e.ephys.Vec2(0, 0));
  e.addParticle(p2);
  let p3 = new e.ephys.Particle();
  p3.setPos(new e.ephys.Vec2(-1, 0));
  e.addParticle(p3);

  let rod = new e.ephys.ParticleRod(1);
  rod.setParticle(0, p1);
  rod.setParticle(1, p2);
  e.addLink(rod);
  let spring32 = new e.ephys.ParticleSpring(p2, 5, 1);
  e.addSpring(p3, spring32);
  let spring23 = new e.ephys.ParticleSpring(p3, 5, 1);
  e.addSpring(p2, spring23);
});
