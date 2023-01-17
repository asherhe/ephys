const ephysWrapper = async function () {
  let start = Date.now();
  const ep = await ephys();
  console.log(`loaded ephys in ${Date.now() - start} ms`);

  const config = {
    // render frame duration in ms
    tickspeed: 1 / 30,
    particleSize: 8,
    hoverParticleSize: 10,
    TAU: 2 * Math.PI,
  };

  /** @type {HTMLCanvasElement} */
  let canvas = document.getElementById("canvas"),
    ctx = canvas.getContext("2d");

  let Camera = {
    x: 0,
    y: 0,
    size: 10,
    toScreenCoords: function (x, y) {
      let scale = canvas.width / Camera.size;
      return { x: (x - Camera.x) * scale + canvas.width / 2, y: (Camera.y - y) * scale + canvas.height / 2 };
    },
    toWorldCoords: function (x, y) {
      let scale = Camera.size / canvas.width;
      return { x: (x - canvas.width / 2) * scale + Camera.x, y: -((y - canvas.height / 2) * scale + Camera.y) };
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
  canvas.addEventListener("mousedown", (e) => {
    if (e.button == 0 && hoverParticle) {
      grabParticle = hoverParticle;
      grabSpring.k = grabParticle.getMass() * 5;
      pWorld.addPFGen(grabParticle, grabSpring);
      canvas.style.cursor = "grabbing";
    }
  });
  canvas.addEventListener("mouseup", (e) => {
    if (e.button == 0 && grabParticle) {
      pWorld.removePFGen(grabParticle, grabSpring);
      grabParticle = false;
      canvas.style.cursor = "initial";
    }
  });
  canvas.addEventListener("mousemove", (e) => {
    mousePos.x = e.offsetX;
    mousePos.y = e.offsetY;
    pos = Camera.toWorldCoords(mousePos.x, mousePos.y);
    mouseWorldPos.set(pos.x, pos.y);
  });

  // function arrow(fromX, fromY, toX, toY, thickness = 1) {
  //   let dx = toX - fromX,
  //     dy = toY - fromY;
  //   let length = Math.sqrt(dx * dx + dy * dy),
  //     headLength = length * 0.2;
  //   thickness *= length * 0.01;
  //   let arrowAngle = Math.atan2(dy, dx),
  //     headAngle = Math.PI / 6,
  //     angleLeft = arrowAngle - headAngle,
  //     angleRight = arrowAngle + headAngle;

  //   ctx.lineWidth = thickness;
  //   ctx.beginPath();
  //   ctx.moveTo(fromX, fromY);
  //   ctx.lineTo(toX, toY);
  //   ctx.stroke();
  //   ctx.beginPath();
  //   ctx.moveTo(toX - headLength * Math.cos(angleLeft), toY - headLength * Math.sin(angleLeft));
  //   ctx.lineTo(toX, toY);
  //   ctx.lineTo(toX - headLength * Math.cos(angleRight), toY - headLength * Math.sin(angleRight));
  //   ctx.stroke();
  // }

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

  // duration of each RENDER frame in seconds
  setInterval(() => {
    pWorld.step(config.tickspeed / 2);
    pWorld.step(config.tickspeed / 2);

    getHoverParticle();
    render();
  }, config.tickspeed * 1000);

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
