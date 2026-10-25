const ephysWrapper = async function () {
  console.log(
    "%cephys\n%ca 2d rigidbody physics engine",
    "color:#f8583e;font-size:4em;font-weight:bold",
    "font-size:1.5em"
  );

  setTimeout(() => {
    $("#load-wait").animate({ opacity: "1" }, { duration: 2000 });
  }, 10000);

  /** @type {HTMLCanvasElement} */
  let canvas = document.getElementById("canvas");
  // right drag pans the camera, so don't open the context menu
  canvas.addEventListener("contextmenu", (e) => e.preventDefault());

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
  // runs the SDL app's main(), which creates its window on #canvas and drives its own frames
  const app = await ephys({ canvas: canvas });
  console.log(`instantiated ephys in ${Date.now() - preloadTimestamp} ms`);

  let load = $("#loading-wrapper");
  load.animate(
    { opacity: 0 },
    {
      duration: 1000,
      complete: () => {
        load.remove();
      },
    }
  );
  $("#canvas").animate({ opacity: 1 }, { duration: 1000 });

  return app;
};

ephysWrapper();
