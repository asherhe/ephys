
var ephys = (() => {
  var _scriptDir = typeof document !== 'undefined' && document.currentScript ? document.currentScript.src : undefined;
  if (typeof __filename !== 'undefined') _scriptDir = _scriptDir || __filename;
  return (
function(ephys) {
  ephys = ephys || {};



// The Module object: Our interface to the outside world. We import
// and export values on it. There are various ways Module can be used:
// 1. Not defined. We create it here
// 2. A function parameter, function(Module) { ..generated code.. }
// 3. pre-run appended it, var Module = {}; ..generated code..
// 4. External script tag defines var Module.
// We need to check if Module already exists (e.g. case 3 above).
// Substitution will be replaced with actual code on later stage of the build,
// this way Closure Compiler will not mangle it (e.g. case 4. above).
// Note that if you want to run closure, and also to use Module
// after the generated code, you will need to define   var Module = {};
// before the code. Then that object will be used in the code, and you
// can continue to use Module afterwards as well.
var Module = typeof ephys != 'undefined' ? ephys : {};

// See https://caniuse.com/mdn-javascript_builtins_object_assign

// Set up the promise that indicates the Module is initialized
var readyPromiseResolve, readyPromiseReject;
Module['ready'] = new Promise(function(resolve, reject) {
  readyPromiseResolve = resolve;
  readyPromiseReject = reject;
});

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_VoidPtr___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_VoidPtr___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_VoidPtr___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_VoidPtr___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_VoidPtr___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_Vec2_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_Vec2_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_Vec2_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_Vec2_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_Vec2_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_Vec2_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_Vec2_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_Vec2_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_Vec2_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_Vec2_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_Vec2_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_Vec2_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_Vec2_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_Vec2_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_Vec2_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_set_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_set_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_set_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_set_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_set_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_norm_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_norm_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_norm_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_norm_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_norm_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_norm2_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_norm2_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_norm2_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_norm2_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_norm2_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_normalize_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_normalize_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_normalize_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_normalize_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_normalize_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_add_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_add_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_add_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_add_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_add_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_sub_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_sub_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_sub_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_sub_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_sub_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_mult_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_mult_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_mult_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_mult_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_mult_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_get_x_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_get_x_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_get_x_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_get_x_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_get_x_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_set_x_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_set_x_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_set_x_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_set_x_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_set_x_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_get_y_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_get_y_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_get_y_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_get_y_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_get_y_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2_set_y_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_set_y_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2_set_y_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2_set_y_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2_set_y_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Vec2___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Vec2___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Vec2___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Vec2___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_Mat2_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_Mat2_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_Mat2_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_Mat2_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_Mat2_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_Mat2_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_Mat2_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_Mat2_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_Mat2_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_Mat2_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_Mat2_4')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_Mat2_4', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_Mat2_4 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_Mat2_4', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_Mat2_4 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_zero_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_zero_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_zero_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_zero_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_zero_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_identity_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_identity_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_identity_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_identity_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_identity_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_at_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_at_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_at_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_at_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_at_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_set_3')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_set_3', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_set_3 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_set_3', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_set_3 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_add_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_add_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_add_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_add_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_add_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_sub_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_sub_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_sub_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_sub_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_sub_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_mult_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_mult_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_mult_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_mult_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_mult_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_determinant_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_determinant_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_determinant_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_determinant_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_determinant_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_inverse_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_inverse_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_inverse_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_inverse_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_inverse_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_invert_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_invert_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_invert_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_invert_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_invert_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_transpose_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_transpose_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_transpose_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_transpose_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_transpose_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_get_data_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_get_data_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_get_data_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_get_data_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_get_data_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2_set_data_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_set_data_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2_set_data_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2_set_data_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2_set_data_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat2___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat2___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat2___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat2___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat3_Mat3_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_Mat3_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat3_Mat3_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_Mat3_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat3_Mat3_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat3_Mat3_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_Mat3_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat3_Mat3_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_Mat3_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat3_Mat3_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat3_Mat3_6')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_Mat3_6', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat3_Mat3_6 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_Mat3_6', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat3_Mat3_6 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat3_identity_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_identity_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat3_identity_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_identity_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat3_identity_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat3_at_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_at_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat3_at_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_at_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat3_at_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat3_set_3')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_set_3', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat3_set_3 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_set_3', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat3_set_3 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat3_determinant_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_determinant_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat3_determinant_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_determinant_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat3_determinant_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat3_inverse_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_inverse_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat3_inverse_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_inverse_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat3_inverse_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat3_invert_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_invert_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat3_invert_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3_invert_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat3_invert_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Mat3___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Mat3___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Mat3___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Mat3___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Collider_origin_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_origin_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Collider_origin_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_origin_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Collider_origin_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Collider_getTransform_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_getTransform_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Collider_getTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_getTransform_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Collider_getTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Collider_getInvTransform_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_getInvTransform_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Collider_getInvTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_getInvTransform_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Collider_getInvTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Collider_setTransform_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_setTransform_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Collider_setTransform_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_setTransform_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Collider_setTransform_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Collider_setInvTransform_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_setInvTransform_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Collider_setInvTransform_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_setInvTransform_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Collider_setInvTransform_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Collider_collider2Object_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_collider2Object_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Collider_collider2Object_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_collider2Object_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Collider_collider2Object_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Collider_object2Collider_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_object2Collider_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Collider_object2Collider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_object2Collider_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Collider_object2Collider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Collider_rotCollider2Object_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_rotCollider2Object_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Collider_rotCollider2Object_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_rotCollider2Object_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Collider_rotCollider2Object_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Collider_rotObject2Collider_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_rotObject2Collider_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Collider_rotObject2Collider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider_rotObject2Collider_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Collider_rotObject2Collider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Collider___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Collider___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Collider___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Collider___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_Rigidbody_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_Rigidbody_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_Rigidbody_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_Rigidbody_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_Rigidbody_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getPos_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getPos_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getPos_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getPos_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getPos_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getVel_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getVel_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getVel_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getVel_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getVel_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getAcc_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getAcc_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getAcc_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getAcc_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getAcc_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getAngle_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getAngle_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getAngle_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getAngle_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getAngle_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getAngVel_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getAngVel_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getAngVel_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getAngVel_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getAngVel_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getMass_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getMass_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getMass_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getMass_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getMass_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getInvMass_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getInvMass_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getInvMass_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getInvMass_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getInvMass_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getInertia_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getInertia_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getInertia_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getInertia_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getInertia_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getInvInertia_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getInvInertia_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getInvInertia_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getInvInertia_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getInvInertia_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getCollider_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getCollider_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getCollider_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getCollider_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getCollider_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getRestitution_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getRestitution_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getRestitution_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getRestitution_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getRestitution_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getTransform_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getTransform_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getTransform_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_getInvTransform_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getInvTransform_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_getInvTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_getInvTransform_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_getInvTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_setPos_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setPos_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_setPos_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setPos_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_setPos_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_setVel_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setVel_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_setVel_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setVel_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_setVel_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_setAcc_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setAcc_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_setAcc_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setAcc_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_setAcc_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_setAngle_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setAngle_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_setAngle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setAngle_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_setAngle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_setAngVel_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setAngVel_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_setAngVel_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setAngVel_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_setAngVel_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_setMass_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setMass_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_setMass_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setMass_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_setMass_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_setInvMass_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setInvMass_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_setInvMass_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setInvMass_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_setInvMass_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_setInertia_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setInertia_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_setInertia_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setInertia_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_setInertia_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_setInvInertia_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setInvInertia_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_setInvInertia_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setInvInertia_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_setInvInertia_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_setCollider_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setCollider_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_setCollider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setCollider_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_setCollider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_setRestitution_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setRestitution_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_setRestitution_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_setRestitution_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_setRestitution_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_world2Local_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_world2Local_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_world2Local_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_world2Local_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_world2Local_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_local2World_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_local2World_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_local2World_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_local2World_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_local2World_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_addForce_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_addForce_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_addForce_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_addForce_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_addForce_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_addForceAt_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_addForceAt_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_addForceAt_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_addForceAt_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_addForceAt_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_addForceAtLocal_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_addForceAtLocal_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_addForceAtLocal_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_addForceAtLocal_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_addForceAtLocal_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_addTorque_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_addTorque_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_addTorque_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_addTorque_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_addTorque_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_clearAccums_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_clearAccums_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_clearAccums_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_clearAccums_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_clearAccums_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody_step_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_step_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody_step_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody_step_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody_step_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Rigidbody___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Rigidbody___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Rigidbody___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Rigidbody___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ContactGenerator_generateContacts_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ContactGenerator_generateContacts_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ContactGenerator_generateContacts_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ContactGenerator_generateContacts_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ContactGenerator_generateContacts_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_CircleCollider_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_CircleCollider_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_CircleCollider_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_CircleCollider_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_CircleCollider_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_CircleCollider_3')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_CircleCollider_3', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_CircleCollider_3 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_CircleCollider_3', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_CircleCollider_3 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_origin_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_origin_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_origin_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_origin_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_origin_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_getTransform_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_getTransform_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_getTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_getTransform_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_getTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_getInvTransform_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_getInvTransform_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_getInvTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_getInvTransform_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_getInvTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_setTransform_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_setTransform_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_setTransform_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_setTransform_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_setTransform_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_setInvTransform_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_setInvTransform_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_setInvTransform_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_setInvTransform_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_setInvTransform_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_collider2Object_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_collider2Object_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_collider2Object_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_collider2Object_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_collider2Object_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_object2Collider_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_object2Collider_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_object2Collider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_object2Collider_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_object2Collider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_rotCollider2Object_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_rotCollider2Object_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_rotCollider2Object_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_rotCollider2Object_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_rotCollider2Object_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_rotObject2Collider_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_rotObject2Collider_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_rotObject2Collider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_rotObject2Collider_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_rotObject2Collider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_get_radius_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_get_radius_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_get_radius_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_get_radius_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_get_radius_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider_set_radius_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_set_radius_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider_set_radius_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider_set_radius_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider_set_radius_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_CircleCollider___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_CircleCollider___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_CircleCollider___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_CircleCollider___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_BoxCollider_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_BoxCollider_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_BoxCollider_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_BoxCollider_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_BoxCollider_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_BoxCollider_3')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_BoxCollider_3', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_BoxCollider_3 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_BoxCollider_3', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_BoxCollider_3 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_origin_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_origin_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_origin_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_origin_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_origin_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_getTransform_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_getTransform_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_getTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_getTransform_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_getTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_getInvTransform_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_getInvTransform_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_getInvTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_getInvTransform_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_getInvTransform_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_setTransform_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_setTransform_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_setTransform_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_setTransform_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_setTransform_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_setInvTransform_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_setInvTransform_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_setInvTransform_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_setInvTransform_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_setInvTransform_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_collider2Object_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_collider2Object_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_collider2Object_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_collider2Object_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_collider2Object_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_object2Collider_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_object2Collider_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_object2Collider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_object2Collider_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_object2Collider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_rotCollider2Object_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_rotCollider2Object_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_rotCollider2Object_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_rotCollider2Object_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_rotCollider2Object_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_rotObject2Collider_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_rotObject2Collider_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_rotObject2Collider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_rotObject2Collider_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_rotObject2Collider_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_get_halfSize_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_get_halfSize_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_get_halfSize_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_get_halfSize_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_get_halfSize_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider_set_halfSize_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_set_halfSize_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider_set_halfSize_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider_set_halfSize_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider_set_halfSize_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_BoxCollider___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_BoxCollider___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_BoxCollider___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_BoxCollider___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_IntersectionDetector_circleCircle_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_IntersectionDetector_circleCircle_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_IntersectionDetector_circleCircle_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_IntersectionDetector_circleCircle_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_IntersectionDetector_circleCircle_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_IntersectionDetector_boxCircle_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_IntersectionDetector_boxCircle_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_IntersectionDetector_boxCircle_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_IntersectionDetector_boxCircle_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_IntersectionDetector_boxCircle_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_IntersectionDetector_boxBox_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_IntersectionDetector_boxBox_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_IntersectionDetector_boxBox_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_IntersectionDetector_boxBox_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_IntersectionDetector_boxBox_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ForceGenerator_updateForce_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ForceGenerator_updateForce_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ForceGenerator_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ForceGenerator_updateForce_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ForceGenerator_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Gravity_Gravity_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Gravity_Gravity_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Gravity_Gravity_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Gravity_Gravity_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Gravity_Gravity_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Gravity_updateForce_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Gravity_updateForce_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Gravity_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Gravity_updateForce_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Gravity_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Gravity___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Gravity___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Gravity___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Gravity___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Gravity___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Spring_Spring_5')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_Spring_5', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Spring_Spring_5 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_Spring_5', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Spring_Spring_5 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Spring_getEnd_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_getEnd_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Spring_getEnd_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_getEnd_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Spring_getEnd_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Spring_setEnd_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_setEnd_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Spring_setEnd_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_setEnd_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Spring_setEnd_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Spring_updateForce_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_updateForce_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Spring_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_updateForce_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Spring_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Spring_get_k_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_get_k_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Spring_get_k_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_get_k_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Spring_get_k_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Spring_set_k_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_set_k_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Spring_set_k_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_set_k_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Spring_set_k_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Spring_get_length_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_get_length_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Spring_get_length_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_get_length_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Spring_get_length_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Spring_set_length_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_set_length_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Spring_set_length_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring_set_length_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Spring_set_length_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Spring___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Spring___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Spring___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Spring___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_World_World_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_World_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_World_World_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_World_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_World_World_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_World_addBody_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_addBody_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_World_addBody_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_addBody_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_World_addBody_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_World_removeBody_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_removeBody_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_World_removeBody_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_removeBody_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_World_removeBody_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_World_addFGen_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_addFGen_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_World_addFGen_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_addFGen_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_World_addFGen_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_World_removeFGen_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_removeFGen_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_World_removeFGen_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_removeFGen_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_World_removeFGen_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_World_addContactGen_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_addContactGen_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_World_addContactGen_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_addContactGen_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_World_addContactGen_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_World_removeContactGen_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_removeContactGen_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_World_removeContactGen_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_removeContactGen_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_World_removeContactGen_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_World_step_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_step_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_World_step_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_World_step_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_World_step_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_World___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_World___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_World___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_World___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_World___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_Particle_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_Particle_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_Particle_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_Particle_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_Particle_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_Particle_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_Particle_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_Particle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_Particle_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_Particle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_Particle_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_Particle_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_Particle_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_Particle_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_Particle_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_getPos_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_getPos_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_getPos_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_getPos_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_getPos_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_getVel_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_getVel_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_getVel_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_getVel_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_getVel_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_getAcc_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_getAcc_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_getAcc_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_getAcc_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_getAcc_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_getMass_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_getMass_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_getMass_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_getMass_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_getMass_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_getInvMass_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_getInvMass_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_getInvMass_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_getInvMass_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_getInvMass_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_getDamping_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_getDamping_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_getDamping_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_getDamping_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_getDamping_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_setPos_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setPos_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_setPos_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setPos_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_setPos_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_setVel_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setVel_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_setVel_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setVel_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_setVel_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_setAcc_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setAcc_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_setAcc_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setAcc_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_setAcc_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_setMass_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setMass_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_setMass_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setMass_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_setMass_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_setInvMass_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setInvMass_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_setInvMass_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setInvMass_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_setInvMass_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_setDamping_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setDamping_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_setDamping_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setDamping_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_setDamping_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_addForce_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_addForce_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_addForce_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_addForce_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_addForce_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_addImpulse_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_addImpulse_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_addImpulse_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_addImpulse_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_addImpulse_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_clearForceAccum_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_clearForceAccum_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_clearForceAccum_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_clearForceAccum_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_clearForceAccum_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_setStatic_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setStatic_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_setStatic_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_setStatic_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_setStatic_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_isStatic_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_isStatic_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_isStatic_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_isStatic_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_isStatic_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle_step_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_step_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle_step_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle_step_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle_step_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_Particle___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_Particle___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_Particle___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_Particle___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleForceGenerator_updateForce_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleForceGenerator_updateForce_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleForceGenerator_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleForceGenerator_updateForce_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleForceGenerator_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleGravity_ParticleGravity_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleGravity_ParticleGravity_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleGravity_ParticleGravity_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleGravity_ParticleGravity_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleGravity_ParticleGravity_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleGravity_updateForce_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleGravity_updateForce_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleGravity_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleGravity_updateForce_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleGravity_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleGravity___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleGravity___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleGravity___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleGravity___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleGravity___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleDrag_ParticleDrag_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleDrag_ParticleDrag_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleDrag_ParticleDrag_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleDrag_ParticleDrag_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleDrag_ParticleDrag_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleDrag_updateForce_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleDrag_updateForce_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleDrag_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleDrag_updateForce_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleDrag_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleDrag___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleDrag___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleDrag___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleDrag___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleDrag___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleSpring_ParticleSpring_3')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_ParticleSpring_3', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleSpring_ParticleSpring_3 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_ParticleSpring_3', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleSpring_ParticleSpring_3 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleSpring_setEnd_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_setEnd_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleSpring_setEnd_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_setEnd_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleSpring_setEnd_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleSpring_getEnd_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_getEnd_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleSpring_getEnd_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_getEnd_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleSpring_getEnd_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleSpring_updateForce_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_updateForce_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleSpring_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_updateForce_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleSpring_updateForce_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleSpring_get_k_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_get_k_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleSpring_get_k_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_get_k_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleSpring_get_k_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleSpring_set_k_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_set_k_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleSpring_set_k_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_set_k_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleSpring_set_k_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleSpring_get_length_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_get_length_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleSpring_get_length_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_get_length_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleSpring_get_length_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleSpring_set_length_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_set_length_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleSpring_set_length_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring_set_length_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleSpring_set_length_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleSpring___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleSpring___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleSpring___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleSpring___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleContactGenerator_generateContacts_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleContactGenerator_generateContacts_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleContactGenerator_generateContacts_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleContactGenerator_generateContacts_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleContactGenerator_generateContacts_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleLink_setParticle_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleLink_setParticle_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleLink_setParticle_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleLink_setParticle_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleLink_setParticle_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleLink_getParticle_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleLink_getParticle_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleLink_getParticle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleLink_getParticle_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleLink_getParticle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleLink_generateContacts_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleLink_generateContacts_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleLink_generateContacts_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleLink_generateContacts_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleLink_generateContacts_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleCable_ParticleCable_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_ParticleCable_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleCable_ParticleCable_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_ParticleCable_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleCable_ParticleCable_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleCable_setParticle_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_setParticle_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleCable_setParticle_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_setParticle_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleCable_setParticle_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleCable_getParticle_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_getParticle_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleCable_getParticle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_getParticle_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleCable_getParticle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleCable_generateContacts_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_generateContacts_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleCable_generateContacts_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_generateContacts_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleCable_generateContacts_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleCable_get_length_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_get_length_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleCable_get_length_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_get_length_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleCable_get_length_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleCable_set_length_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_set_length_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleCable_set_length_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_set_length_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleCable_set_length_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleCable_get_restitution_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_get_restitution_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleCable_get_restitution_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_get_restitution_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleCable_get_restitution_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleCable_set_restitution_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_set_restitution_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleCable_set_restitution_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable_set_restitution_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleCable_set_restitution_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleCable___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleCable___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleCable___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleCable___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleRod_ParticleRod_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod_ParticleRod_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleRod_ParticleRod_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod_ParticleRod_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleRod_ParticleRod_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleRod_setParticle_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod_setParticle_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleRod_setParticle_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod_setParticle_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleRod_setParticle_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleRod_getParticle_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod_getParticle_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleRod_getParticle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod_getParticle_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleRod_getParticle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleRod_generateContacts_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod_generateContacts_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleRod_generateContacts_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod_generateContacts_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleRod_generateContacts_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleRod_get_length_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod_get_length_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleRod_get_length_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod_get_length_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleRod_get_length_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleRod_set_length_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod_set_length_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleRod_set_length_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod_set_length_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleRod_set_length_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleRod___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleRod___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleRod___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleRod___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleWorld_ParticleWorld_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_ParticleWorld_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleWorld_ParticleWorld_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_ParticleWorld_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleWorld_ParticleWorld_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleWorld_ParticleWorld_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_ParticleWorld_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleWorld_ParticleWorld_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_ParticleWorld_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleWorld_ParticleWorld_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleWorld_addParticle_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_addParticle_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleWorld_addParticle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_addParticle_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleWorld_addParticle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleWorld_removeParticle_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_removeParticle_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleWorld_removeParticle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_removeParticle_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleWorld_removeParticle_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleWorld_addPFGen_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_addPFGen_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleWorld_addPFGen_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_addPFGen_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleWorld_addPFGen_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleWorld_removePFGen_2')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_removePFGen_2', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleWorld_removePFGen_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_removePFGen_2', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleWorld_removePFGen_2 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleWorld_addPContactGenerator_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_addPContactGenerator_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleWorld_addPContactGenerator_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_addPContactGenerator_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleWorld_addPContactGenerator_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleWorld_removePContactGenerator_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_removePContactGenerator_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleWorld_removePContactGenerator_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_removePContactGenerator_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleWorld_removePContactGenerator_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleWorld_generateContacts_0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_generateContacts_0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleWorld_generateContacts_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_generateContacts_0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleWorld_generateContacts_0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleWorld_step_1')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_step_1', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleWorld_step_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld_step_1', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleWorld_step_1 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_emscripten_bind_ParticleWorld___destroy___0')) {
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld___destroy___0', { configurable: true, get: function() { abort('You are getting _emscripten_bind_ParticleWorld___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_emscripten_bind_ParticleWorld___destroy___0', { configurable: true, set: function() { abort('You are setting _emscripten_bind_ParticleWorld___destroy___0 on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], '_fflush')) {
        Object.defineProperty(Module['ready'], '_fflush', { configurable: true, get: function() { abort('You are getting _fflush on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], '_fflush', { configurable: true, set: function() { abort('You are setting _fflush on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

      if (!Object.getOwnPropertyDescriptor(Module['ready'], 'onRuntimeInitialized')) {
        Object.defineProperty(Module['ready'], 'onRuntimeInitialized', { configurable: true, get: function() { abort('You are getting onRuntimeInitialized on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
        Object.defineProperty(Module['ready'], 'onRuntimeInitialized', { configurable: true, set: function() { abort('You are setting onRuntimeInitialized on the Promise object, instead of the instance. Use .then() to get called back with the instance, see the MODULARIZE docs in src/settings.js') } });
      }
    

// --pre-jses are emitted after the Module integration code, so that they can
// refer to Module (if they choose; they can also define Module)


// Sometimes an existing Module object exists with properties
// meant to overwrite the default module functionality. Here
// we collect those properties and reapply _after_ we configure
// the current environment's defaults to avoid having to be so
// defensive during initialization.
var moduleOverrides = Object.assign({}, Module);

var arguments_ = [];
var thisProgram = './this.program';
var quit_ = (status, toThrow) => {
  throw toThrow;
};

// Determine the runtime environment we are in. You can customize this by
// setting the ENVIRONMENT setting at compile time (see settings.js).

// Attempt to auto-detect the environment
var ENVIRONMENT_IS_WEB = typeof window == 'object';
var ENVIRONMENT_IS_WORKER = typeof importScripts == 'function';
// N.b. Electron.js environment is simultaneously a NODE-environment, but
// also a web environment.
var ENVIRONMENT_IS_NODE = typeof process == 'object' && typeof process.versions == 'object' && typeof process.versions.node == 'string';
var ENVIRONMENT_IS_SHELL = !ENVIRONMENT_IS_WEB && !ENVIRONMENT_IS_NODE && !ENVIRONMENT_IS_WORKER;

if (Module['ENVIRONMENT']) {
  throw new Error('Module.ENVIRONMENT has been deprecated. To force the environment, use the ENVIRONMENT compile-time option (for example, -sENVIRONMENT=web or -sENVIRONMENT=node)');
}

// `/` should be present at the end if `scriptDirectory` is not empty
var scriptDirectory = '';
function locateFile(path) {
  if (Module['locateFile']) {
    return Module['locateFile'](path, scriptDirectory);
  }
  return scriptDirectory + path;
}

// Hooks that are implemented differently in different runtime environments.
var read_,
    readAsync,
    readBinary,
    setWindowTitle;

// Normally we don't log exceptions but instead let them bubble out the top
// level where the embedding environment (e.g. the browser) can handle
// them.
// However under v8 and node we sometimes exit the process direcly in which case
// its up to use us to log the exception before exiting.
// If we fix https://github.com/emscripten-core/emscripten/issues/15080
// this may no longer be needed under node.
function logExceptionOnExit(e) {
  if (e instanceof ExitStatus) return;
  let toLog = e;
  if (e && typeof e == 'object' && e.stack) {
    toLog = [e, e.stack];
  }
  err('exiting due to exception: ' + toLog);
}

var fs;
var nodePath;
var requireNodeFS;

if (ENVIRONMENT_IS_NODE) {
  if (!(typeof process == 'object' && typeof require == 'function')) throw new Error('not compiled for this environment (did you build to HTML and try to run it not on the web, or set ENVIRONMENT to something - like node - and run it someplace else - like on the web?)');
  if (ENVIRONMENT_IS_WORKER) {
    scriptDirectory = require('path').dirname(scriptDirectory) + '/';
  } else {
    scriptDirectory = __dirname + '/';
  }

// include: node_shell_read.js


requireNodeFS = () => {
  // Use nodePath as the indicator for these not being initialized,
  // since in some environments a global fs may have already been
  // created.
  if (!nodePath) {
    fs = require('fs');
    nodePath = require('path');
  }
};

read_ = function shell_read(filename, binary) {
  requireNodeFS();
  filename = nodePath['normalize'](filename);
  return fs.readFileSync(filename, binary ? undefined : 'utf8');
};

readBinary = (filename) => {
  var ret = read_(filename, true);
  if (!ret.buffer) {
    ret = new Uint8Array(ret);
  }
  assert(ret.buffer);
  return ret;
};

readAsync = (filename, onload, onerror) => {
  requireNodeFS();
  filename = nodePath['normalize'](filename);
  fs.readFile(filename, function(err, data) {
    if (err) onerror(err);
    else onload(data.buffer);
  });
};

// end include: node_shell_read.js
  if (process['argv'].length > 1) {
    thisProgram = process['argv'][1].replace(/\\/g, '/');
  }

  arguments_ = process['argv'].slice(2);

  // MODULARIZE will export the module in the proper place outside, we don't need to export here

  process['on']('uncaughtException', function(ex) {
    // suppress ExitStatus exceptions from showing an error
    if (!(ex instanceof ExitStatus)) {
      throw ex;
    }
  });

  // Without this older versions of node (< v15) will log unhandled rejections
  // but return 0, which is not normally the desired behaviour.  This is
  // not be needed with node v15 and about because it is now the default
  // behaviour:
  // See https://nodejs.org/api/cli.html#cli_unhandled_rejections_mode
  process['on']('unhandledRejection', function(reason) { throw reason; });

  quit_ = (status, toThrow) => {
    if (keepRuntimeAlive()) {
      process['exitCode'] = status;
      throw toThrow;
    }
    logExceptionOnExit(toThrow);
    process['exit'](status);
  };

  Module['inspect'] = function () { return '[Emscripten Module object]'; };

} else
if (ENVIRONMENT_IS_SHELL) {

  if ((typeof process == 'object' && typeof require === 'function') || typeof window == 'object' || typeof importScripts == 'function') throw new Error('not compiled for this environment (did you build to HTML and try to run it not on the web, or set ENVIRONMENT to something - like node - and run it someplace else - like on the web?)');

  if (typeof read != 'undefined') {
    read_ = function shell_read(f) {
      return read(f);
    };
  }

  readBinary = function readBinary(f) {
    let data;
    if (typeof readbuffer == 'function') {
      return new Uint8Array(readbuffer(f));
    }
    data = read(f, 'binary');
    assert(typeof data == 'object');
    return data;
  };

  readAsync = function readAsync(f, onload, onerror) {
    setTimeout(() => onload(readBinary(f)), 0);
  };

  if (typeof scriptArgs != 'undefined') {
    arguments_ = scriptArgs;
  } else if (typeof arguments != 'undefined') {
    arguments_ = arguments;
  }

  if (typeof quit == 'function') {
    quit_ = (status, toThrow) => {
      logExceptionOnExit(toThrow);
      quit(status);
    };
  }

  if (typeof print != 'undefined') {
    // Prefer to use print/printErr where they exist, as they usually work better.
    if (typeof console == 'undefined') console = /** @type{!Console} */({});
    console.log = /** @type{!function(this:Console, ...*): undefined} */ (print);
    console.warn = console.error = /** @type{!function(this:Console, ...*): undefined} */ (typeof printErr != 'undefined' ? printErr : print);
  }

} else

// Note that this includes Node.js workers when relevant (pthreads is enabled).
// Node.js workers are detected as a combination of ENVIRONMENT_IS_WORKER and
// ENVIRONMENT_IS_NODE.
if (ENVIRONMENT_IS_WEB || ENVIRONMENT_IS_WORKER) {
  if (ENVIRONMENT_IS_WORKER) { // Check worker, not web, since window could be polyfilled
    scriptDirectory = self.location.href;
  } else if (typeof document != 'undefined' && document.currentScript) { // web
    scriptDirectory = document.currentScript.src;
  }
  // When MODULARIZE, this JS may be executed later, after document.currentScript
  // is gone, so we saved it, and we use it here instead of any other info.
  if (_scriptDir) {
    scriptDirectory = _scriptDir;
  }
  // blob urls look like blob:http://site.com/etc/etc and we cannot infer anything from them.
  // otherwise, slice off the final part of the url to find the script directory.
  // if scriptDirectory does not contain a slash, lastIndexOf will return -1,
  // and scriptDirectory will correctly be replaced with an empty string.
  // If scriptDirectory contains a query (starting with ?) or a fragment (starting with #),
  // they are removed because they could contain a slash.
  if (scriptDirectory.indexOf('blob:') !== 0) {
    scriptDirectory = scriptDirectory.substr(0, scriptDirectory.replace(/[?#].*/, "").lastIndexOf('/')+1);
  } else {
    scriptDirectory = '';
  }

  if (!(typeof window == 'object' || typeof importScripts == 'function')) throw new Error('not compiled for this environment (did you build to HTML and try to run it not on the web, or set ENVIRONMENT to something - like node - and run it someplace else - like on the web?)');

  // Differentiate the Web Worker from the Node Worker case, as reading must
  // be done differently.
  {
// include: web_or_worker_shell_read.js


  read_ = (url) => {
      var xhr = new XMLHttpRequest();
      xhr.open('GET', url, false);
      xhr.send(null);
      return xhr.responseText;
  }

  if (ENVIRONMENT_IS_WORKER) {
    readBinary = (url) => {
        var xhr = new XMLHttpRequest();
        xhr.open('GET', url, false);
        xhr.responseType = 'arraybuffer';
        xhr.send(null);
        return new Uint8Array(/** @type{!ArrayBuffer} */(xhr.response));
    };
  }

  readAsync = (url, onload, onerror) => {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', url, true);
    xhr.responseType = 'arraybuffer';
    xhr.onload = () => {
      if (xhr.status == 200 || (xhr.status == 0 && xhr.response)) { // file URLs can return 0
        onload(xhr.response);
        return;
      }
      onerror();
    };
    xhr.onerror = onerror;
    xhr.send(null);
  }

// end include: web_or_worker_shell_read.js
  }

  setWindowTitle = (title) => document.title = title;
} else
{
  throw new Error('environment detection error');
}

var out = Module['print'] || console.log.bind(console);
var err = Module['printErr'] || console.warn.bind(console);

// Merge back in the overrides
Object.assign(Module, moduleOverrides);
// Free the object hierarchy contained in the overrides, this lets the GC
// reclaim data used e.g. in memoryInitializerRequest, which is a large typed array.
moduleOverrides = null;
checkIncomingModuleAPI();

// Emit code to handle expected values on the Module object. This applies Module.x
// to the proper local x. This has two benefits: first, we only emit it if it is
// expected to arrive, and second, by using a local everywhere else that can be
// minified.

if (Module['arguments']) arguments_ = Module['arguments'];legacyModuleProp('arguments', 'arguments_');

if (Module['thisProgram']) thisProgram = Module['thisProgram'];legacyModuleProp('thisProgram', 'thisProgram');

if (Module['quit']) quit_ = Module['quit'];legacyModuleProp('quit', 'quit_');

// perform assertions in shell.js after we set up out() and err(), as otherwise if an assertion fails it cannot print the message
// Assertions on removed incoming Module JS APIs.
assert(typeof Module['memoryInitializerPrefixURL'] == 'undefined', 'Module.memoryInitializerPrefixURL option was removed, use Module.locateFile instead');
assert(typeof Module['pthreadMainPrefixURL'] == 'undefined', 'Module.pthreadMainPrefixURL option was removed, use Module.locateFile instead');
assert(typeof Module['cdInitializerPrefixURL'] == 'undefined', 'Module.cdInitializerPrefixURL option was removed, use Module.locateFile instead');
assert(typeof Module['filePackagePrefixURL'] == 'undefined', 'Module.filePackagePrefixURL option was removed, use Module.locateFile instead');
assert(typeof Module['read'] == 'undefined', 'Module.read option was removed (modify read_ in JS)');
assert(typeof Module['readAsync'] == 'undefined', 'Module.readAsync option was removed (modify readAsync in JS)');
assert(typeof Module['readBinary'] == 'undefined', 'Module.readBinary option was removed (modify readBinary in JS)');
assert(typeof Module['setWindowTitle'] == 'undefined', 'Module.setWindowTitle option was removed (modify setWindowTitle in JS)');
assert(typeof Module['TOTAL_MEMORY'] == 'undefined', 'Module.TOTAL_MEMORY has been renamed Module.INITIAL_MEMORY');
legacyModuleProp('read', 'read_');
legacyModuleProp('readAsync', 'readAsync');
legacyModuleProp('readBinary', 'readBinary');
legacyModuleProp('setWindowTitle', 'setWindowTitle');
var IDBFS = 'IDBFS is no longer included by default; build with -lidbfs.js';
var PROXYFS = 'PROXYFS is no longer included by default; build with -lproxyfs.js';
var WORKERFS = 'WORKERFS is no longer included by default; build with -lworkerfs.js';
var NODEFS = 'NODEFS is no longer included by default; build with -lnodefs.js';
function alignMemory() { abort('`alignMemory` is now a library function and not included by default; add it to your library.js __deps or to DEFAULT_LIBRARY_FUNCS_TO_INCLUDE on the command line'); }

assert(!ENVIRONMENT_IS_SHELL, "shell environment detected but not enabled at build time.  Add 'shell' to `-sENVIRONMENT` to enable.");




var STACK_ALIGN = 16;
var POINTER_SIZE = 4;

function getNativeTypeSize(type) {
  switch (type) {
    case 'i1': case 'i8': case 'u8': return 1;
    case 'i16': case 'u16': return 2;
    case 'i32': case 'u32': return 4;
    case 'i64': case 'u64': return 8;
    case 'float': return 4;
    case 'double': return 8;
    default: {
      if (type[type.length - 1] === '*') {
        return POINTER_SIZE;
      } else if (type[0] === 'i') {
        const bits = Number(type.substr(1));
        assert(bits % 8 === 0, 'getNativeTypeSize invalid bits ' + bits + ', type ' + type);
        return bits / 8;
      } else {
        return 0;
      }
    }
  }
}

function warnOnce(text) {
  if (!warnOnce.shown) warnOnce.shown = {};
  if (!warnOnce.shown[text]) {
    warnOnce.shown[text] = 1;
    err(text);
  }
}

// include: runtime_functions.js


// This gives correct answers for everything less than 2^{14} = 16384
// I hope nobody is contemplating functions with 16384 arguments...
function uleb128Encode(n) {
  assert(n < 16384);
  if (n < 128) {
    return [n];
  }
  return [(n % 128) | 128, n >> 7];
}

// Wraps a JS function as a wasm function with a given signature.
function convertJsFunctionToWasm(func, sig) {

  // If the type reflection proposal is available, use the new
  // "WebAssembly.Function" constructor.
  // Otherwise, construct a minimal wasm module importing the JS function and
  // re-exporting it.
  if (typeof WebAssembly.Function == "function") {
    var typeNames = {
      'i': 'i32',
      'j': 'i64',
      'f': 'f32',
      'd': 'f64',
      'p': 'i32',
    };
    var type = {
      parameters: [],
      results: sig[0] == 'v' ? [] : [typeNames[sig[0]]]
    };
    for (var i = 1; i < sig.length; ++i) {
      assert(sig[i] in typeNames, 'invalid signature char: ' + sig[i]);
      type.parameters.push(typeNames[sig[i]]);
    }
    return new WebAssembly.Function(type, func);
  }

  // The module is static, with the exception of the type section, which is
  // generated based on the signature passed in.
  var typeSection = [
    0x01, // count: 1
    0x60, // form: func
  ];
  var sigRet = sig.slice(0, 1);
  var sigParam = sig.slice(1);
  var typeCodes = {
    'i': 0x7f, // i32
    'p': 0x7f, // i32
    'j': 0x7e, // i64
    'f': 0x7d, // f32
    'd': 0x7c, // f64
  };

  // Parameters, length + signatures
  typeSection = typeSection.concat(uleb128Encode(sigParam.length));
  for (var i = 0; i < sigParam.length; ++i) {
    assert(sigParam[i] in typeCodes, 'invalid signature char: ' + sigParam[i]);
    typeSection.push(typeCodes[sigParam[i]]);
  }

  // Return values, length + signatures
  // With no multi-return in MVP, either 0 (void) or 1 (anything else)
  if (sigRet == 'v') {
    typeSection.push(0x00);
  } else {
    typeSection = typeSection.concat([0x01, typeCodes[sigRet]]);
  }

  // Write the section code and overall length of the type section into the
  // section header
  typeSection = [0x01 /* Type section code */].concat(
    uleb128Encode(typeSection.length),
    typeSection
  );

  // Rest of the module is static
  var bytes = new Uint8Array([
    0x00, 0x61, 0x73, 0x6d, // magic ("\0asm")
    0x01, 0x00, 0x00, 0x00, // version: 1
  ].concat(typeSection, [
    0x02, 0x07, // import section
      // (import "e" "f" (func 0 (type 0)))
      0x01, 0x01, 0x65, 0x01, 0x66, 0x00, 0x00,
    0x07, 0x05, // export section
      // (export "f" (func 0 (type 0)))
      0x01, 0x01, 0x66, 0x00, 0x00,
  ]));

   // We can compile this wasm module synchronously because it is very small.
  // This accepts an import (at "e.f"), that it reroutes to an export (at "f")
  var module = new WebAssembly.Module(bytes);
  var instance = new WebAssembly.Instance(module, {
    'e': {
      'f': func
    }
  });
  var wrappedFunc = instance.exports['f'];
  return wrappedFunc;
}

var freeTableIndexes = [];

// Weak map of functions in the table to their indexes, created on first use.
var functionsInTableMap;

function getEmptyTableSlot() {
  // Reuse a free index if there is one, otherwise grow.
  if (freeTableIndexes.length) {
    return freeTableIndexes.pop();
  }
  // Grow the table
  try {
    wasmTable.grow(1);
  } catch (err) {
    if (!(err instanceof RangeError)) {
      throw err;
    }
    throw 'Unable to grow wasm table. Set ALLOW_TABLE_GROWTH.';
  }
  return wasmTable.length - 1;
}

function updateTableMap(offset, count) {
  for (var i = offset; i < offset + count; i++) {
    var item = getWasmTableEntry(i);
    // Ignore null values.
    if (item) {
      functionsInTableMap.set(item, i);
    }
  }
}

/**
 * Add a function to the table.
 * 'sig' parameter is required if the function being added is a JS function.
 * @param {string=} sig
 */
function addFunction(func, sig) {
  assert(typeof func != 'undefined');

  // Check if the function is already in the table, to ensure each function
  // gets a unique index. First, create the map if this is the first use.
  if (!functionsInTableMap) {
    functionsInTableMap = new WeakMap();
    updateTableMap(0, wasmTable.length);
  }
  if (functionsInTableMap.has(func)) {
    return functionsInTableMap.get(func);
  }

  // It's not in the table, add it now.

  var ret = getEmptyTableSlot();

  // Set the new value.
  try {
    // Attempting to call this with JS function will cause of table.set() to fail
    setWasmTableEntry(ret, func);
  } catch (err) {
    if (!(err instanceof TypeError)) {
      throw err;
    }
    assert(typeof sig != 'undefined', 'Missing signature argument to addFunction: ' + func);
    var wrapped = convertJsFunctionToWasm(func, sig);
    setWasmTableEntry(ret, wrapped);
  }

  functionsInTableMap.set(func, ret);

  return ret;
}

function removeFunction(index) {
  functionsInTableMap.delete(getWasmTableEntry(index));
  freeTableIndexes.push(index);
}

// end include: runtime_functions.js
// include: runtime_debug.js


function legacyModuleProp(prop, newName) {
  if (!Object.getOwnPropertyDescriptor(Module, prop)) {
    Object.defineProperty(Module, prop, {
      configurable: true,
      get: function() {
        abort('Module.' + prop + ' has been replaced with plain ' + newName + ' (the initial value can be provided on Module, but after startup the value is only looked for on a local variable of that name)');
      }
    });
  }
}

function ignoredModuleProp(prop) {
  if (Object.getOwnPropertyDescriptor(Module, prop)) {
    abort('`Module.' + prop + '` was supplied but `' + prop + '` not included in INCOMING_MODULE_JS_API');
  }
}

function unexportedMessage(sym, isFSSybol) {
  var msg = "'" + sym + "' was not exported. add it to EXPORTED_RUNTIME_METHODS (see the FAQ)";
  if (isFSSybol) {
    msg += '. Alternatively, forcing filesystem support (-sFORCE_FILESYSTEM) can export this for you';
  }
  return msg;
}

function unexportedRuntimeSymbol(sym, isFSSybol) {
  if (!Object.getOwnPropertyDescriptor(Module, sym)) {
    Object.defineProperty(Module, sym, {
      configurable: true,
      get: function() {
        abort(unexportedMessage(sym, isFSSybol));
      }
    });
  }
}

function unexportedRuntimeFunction(sym, isFSSybol) {
  if (!Object.getOwnPropertyDescriptor(Module, sym)) {
    Module[sym] = () => abort(unexportedMessage(sym, isFSSybol));
  }
}

// end include: runtime_debug.js
var tempRet0 = 0;
var setTempRet0 = (value) => { tempRet0 = value; };
var getTempRet0 = () => tempRet0;



// === Preamble library stuff ===

// Documentation for the public APIs defined in this file must be updated in:
//    site/source/docs/api_reference/preamble.js.rst
// A prebuilt local version of the documentation is available at:
//    site/build/text/docs/api_reference/preamble.js.txt
// You can also build docs locally as HTML or other formats in site/
// An online HTML version (which may be of a different version of Emscripten)
//    is up at http://kripken.github.io/emscripten-site/docs/api_reference/preamble.js.html

var wasmBinary;
if (Module['wasmBinary']) wasmBinary = Module['wasmBinary'];legacyModuleProp('wasmBinary', 'wasmBinary');
var noExitRuntime = Module['noExitRuntime'] || true;legacyModuleProp('noExitRuntime', 'noExitRuntime');

if (typeof WebAssembly != 'object') {
  abort('no native wasm support detected');
}

// Wasm globals

var wasmMemory;

//========================================
// Runtime essentials
//========================================

// whether we are quitting the application. no code should run after this.
// set in exit() and abort()
var ABORT = false;

// set by exit() and abort().  Passed to 'onExit' handler.
// NOTE: This is also used as the process return code code in shell environments
// but only when noExitRuntime is false.
var EXITSTATUS;

/** @type {function(*, string=)} */
function assert(condition, text) {
  if (!condition) {
    abort('Assertion failed' + (text ? ': ' + text : ''));
  }
}

// Returns the C function with a specified identifier (for C++, you need to do manual name mangling)
function getCFunc(ident) {
  var func = Module['_' + ident]; // closure exported function
  assert(func, 'Cannot call unknown function ' + ident + ', make sure it is exported');
  return func;
}

// C calling interface.
/** @param {string|null=} returnType
    @param {Array=} argTypes
    @param {Arguments|Array=} args
    @param {Object=} opts */
function ccall(ident, returnType, argTypes, args, opts) {
  // For fast lookup of conversion functions
  var toC = {
    'string': function(str) {
      var ret = 0;
      if (str !== null && str !== undefined && str !== 0) { // null string
        // at most 4 bytes per UTF-8 code point, +1 for the trailing '\0'
        var len = (str.length << 2) + 1;
        ret = stackAlloc(len);
        stringToUTF8(str, ret, len);
      }
      return ret;
    },
    'array': function(arr) {
      var ret = stackAlloc(arr.length);
      writeArrayToMemory(arr, ret);
      return ret;
    }
  };

  function convertReturnValue(ret) {
    if (returnType === 'string') {
      
      return UTF8ToString(ret);
    }
    if (returnType === 'boolean') return Boolean(ret);
    return ret;
  }

  var func = getCFunc(ident);
  var cArgs = [];
  var stack = 0;
  assert(returnType !== 'array', 'Return type should not be "array".');
  if (args) {
    for (var i = 0; i < args.length; i++) {
      var converter = toC[argTypes[i]];
      if (converter) {
        if (stack === 0) stack = stackSave();
        cArgs[i] = converter(args[i]);
      } else {
        cArgs[i] = args[i];
      }
    }
  }
  var ret = func.apply(null, cArgs);
  function onDone(ret) {
    if (stack !== 0) stackRestore(stack);
    return convertReturnValue(ret);
  }

  ret = onDone(ret);
  return ret;
}

/** @param {string=} returnType
    @param {Array=} argTypes
    @param {Object=} opts */
function cwrap(ident, returnType, argTypes, opts) {
  return function() {
    return ccall(ident, returnType, argTypes, arguments, opts);
  }
}

// We used to include malloc/free by default in the past. Show a helpful error in
// builds with assertions.
function _free() {
  // Show a helpful error since we used to include free by default in the past.
  abort("free() called but not included in the build - add '_free' to EXPORTED_FUNCTIONS");
}

// include: runtime_legacy.js


var ALLOC_NORMAL = 0; // Tries to use _malloc()
var ALLOC_STACK = 1; // Lives for the duration of the current function call

/**
 * allocate(): This function is no longer used by emscripten but is kept around to avoid
 *             breaking external users.
 *             You should normally not use allocate(), and instead allocate
 *             memory using _malloc()/stackAlloc(), initialize it with
 *             setValue(), and so forth.
 * @param {(Uint8Array|Array<number>)} slab: An array of data.
 * @param {number=} allocator : How to allocate memory, see ALLOC_*
 */
function allocate(slab, allocator) {
  var ret;
  assert(typeof allocator == 'number', 'allocate no longer takes a type argument')
  assert(typeof slab != 'number', 'allocate no longer takes a number as arg0')

  if (allocator == ALLOC_STACK) {
    ret = stackAlloc(slab.length);
  } else {
    ret = _malloc(slab.length);
  }

  if (!slab.subarray && !slab.slice) {
    slab = new Uint8Array(slab);
  }
  HEAPU8.set(slab, ret);
  return ret;
}

// end include: runtime_legacy.js
// include: runtime_strings.js


// runtime_strings.js: Strings related runtime functions that are part of both MINIMAL_RUNTIME and regular runtime.

var UTF8Decoder = typeof TextDecoder != 'undefined' ? new TextDecoder('utf8') : undefined;

// Given a pointer 'ptr' to a null-terminated UTF8-encoded string in the given array that contains uint8 values, returns
// a copy of that string as a Javascript String object.
/**
 * heapOrArray is either a regular array, or a JavaScript typed array view.
 * @param {number} idx
 * @param {number=} maxBytesToRead
 * @return {string}
 */
function UTF8ArrayToString(heapOrArray, idx, maxBytesToRead) {
  var endIdx = idx + maxBytesToRead;
  var endPtr = idx;
  // TextDecoder needs to know the byte length in advance, it doesn't stop on null terminator by itself.
  // Also, use the length info to avoid running tiny strings through TextDecoder, since .subarray() allocates garbage.
  // (As a tiny code save trick, compare endPtr against endIdx using a negation, so that undefined means Infinity)
  while (heapOrArray[endPtr] && !(endPtr >= endIdx)) ++endPtr;

  if (endPtr - idx > 16 && heapOrArray.buffer && UTF8Decoder) {
    return UTF8Decoder.decode(heapOrArray.subarray(idx, endPtr));
  } else {
    var str = '';
    // If building with TextDecoder, we have already computed the string length above, so test loop end condition against that
    while (idx < endPtr) {
      // For UTF8 byte structure, see:
      // http://en.wikipedia.org/wiki/UTF-8#Description
      // https://www.ietf.org/rfc/rfc2279.txt
      // https://tools.ietf.org/html/rfc3629
      var u0 = heapOrArray[idx++];
      if (!(u0 & 0x80)) { str += String.fromCharCode(u0); continue; }
      var u1 = heapOrArray[idx++] & 63;
      if ((u0 & 0xE0) == 0xC0) { str += String.fromCharCode(((u0 & 31) << 6) | u1); continue; }
      var u2 = heapOrArray[idx++] & 63;
      if ((u0 & 0xF0) == 0xE0) {
        u0 = ((u0 & 15) << 12) | (u1 << 6) | u2;
      } else {
        if ((u0 & 0xF8) != 0xF0) warnOnce('Invalid UTF-8 leading byte 0x' + u0.toString(16) + ' encountered when deserializing a UTF-8 string in wasm memory to a JS string!');
        u0 = ((u0 & 7) << 18) | (u1 << 12) | (u2 << 6) | (heapOrArray[idx++] & 63);
      }

      if (u0 < 0x10000) {
        str += String.fromCharCode(u0);
      } else {
        var ch = u0 - 0x10000;
        str += String.fromCharCode(0xD800 | (ch >> 10), 0xDC00 | (ch & 0x3FF));
      }
    }
  }
  return str;
}

// Given a pointer 'ptr' to a null-terminated UTF8-encoded string in the emscripten HEAP, returns a
// copy of that string as a Javascript String object.
// maxBytesToRead: an optional length that specifies the maximum number of bytes to read. You can omit
//                 this parameter to scan the string until the first \0 byte. If maxBytesToRead is
//                 passed, and the string at [ptr, ptr+maxBytesToReadr[ contains a null byte in the
//                 middle, then the string will cut short at that byte index (i.e. maxBytesToRead will
//                 not produce a string of exact length [ptr, ptr+maxBytesToRead[)
//                 N.B. mixing frequent uses of UTF8ToString() with and without maxBytesToRead may
//                 throw JS JIT optimizations off, so it is worth to consider consistently using one
//                 style or the other.
/**
 * @param {number} ptr
 * @param {number=} maxBytesToRead
 * @return {string}
 */
function UTF8ToString(ptr, maxBytesToRead) {
  return ptr ? UTF8ArrayToString(HEAPU8, ptr, maxBytesToRead) : '';
}

// Copies the given Javascript String object 'str' to the given byte array at address 'outIdx',
// encoded in UTF8 form and null-terminated. The copy will require at most str.length*4+1 bytes of space in the HEAP.
// Use the function lengthBytesUTF8 to compute the exact number of bytes (excluding null terminator) that this function will write.
// Parameters:
//   str: the Javascript string to copy.
//   heap: the array to copy to. Each index in this array is assumed to be one 8-byte element.
//   outIdx: The starting offset in the array to begin the copying.
//   maxBytesToWrite: The maximum number of bytes this function can write to the array.
//                    This count should include the null terminator,
//                    i.e. if maxBytesToWrite=1, only the null terminator will be written and nothing else.
//                    maxBytesToWrite=0 does not write any bytes to the output, not even the null terminator.
// Returns the number of bytes written, EXCLUDING the null terminator.

function stringToUTF8Array(str, heap, outIdx, maxBytesToWrite) {
  if (!(maxBytesToWrite > 0)) // Parameter maxBytesToWrite is not optional. Negative values, 0, null, undefined and false each don't write out any bytes.
    return 0;

  var startIdx = outIdx;
  var endIdx = outIdx + maxBytesToWrite - 1; // -1 for string null terminator.
  for (var i = 0; i < str.length; ++i) {
    // Gotcha: charCodeAt returns a 16-bit word that is a UTF-16 encoded code unit, not a Unicode code point of the character! So decode UTF16->UTF32->UTF8.
    // See http://unicode.org/faq/utf_bom.html#utf16-3
    // For UTF8 byte structure, see http://en.wikipedia.org/wiki/UTF-8#Description and https://www.ietf.org/rfc/rfc2279.txt and https://tools.ietf.org/html/rfc3629
    var u = str.charCodeAt(i); // possibly a lead surrogate
    if (u >= 0xD800 && u <= 0xDFFF) {
      var u1 = str.charCodeAt(++i);
      u = 0x10000 + ((u & 0x3FF) << 10) | (u1 & 0x3FF);
    }
    if (u <= 0x7F) {
      if (outIdx >= endIdx) break;
      heap[outIdx++] = u;
    } else if (u <= 0x7FF) {
      if (outIdx + 1 >= endIdx) break;
      heap[outIdx++] = 0xC0 | (u >> 6);
      heap[outIdx++] = 0x80 | (u & 63);
    } else if (u <= 0xFFFF) {
      if (outIdx + 2 >= endIdx) break;
      heap[outIdx++] = 0xE0 | (u >> 12);
      heap[outIdx++] = 0x80 | ((u >> 6) & 63);
      heap[outIdx++] = 0x80 | (u & 63);
    } else {
      if (outIdx + 3 >= endIdx) break;
      if (u > 0x10FFFF) warnOnce('Invalid Unicode code point 0x' + u.toString(16) + ' encountered when serializing a JS string to a UTF-8 string in wasm memory! (Valid unicode code points should be in range 0-0x10FFFF).');
      heap[outIdx++] = 0xF0 | (u >> 18);
      heap[outIdx++] = 0x80 | ((u >> 12) & 63);
      heap[outIdx++] = 0x80 | ((u >> 6) & 63);
      heap[outIdx++] = 0x80 | (u & 63);
    }
  }
  // Null-terminate the pointer to the buffer.
  heap[outIdx] = 0;
  return outIdx - startIdx;
}

// Copies the given Javascript String object 'str' to the emscripten HEAP at address 'outPtr',
// null-terminated and encoded in UTF8 form. The copy will require at most str.length*4+1 bytes of space in the HEAP.
// Use the function lengthBytesUTF8 to compute the exact number of bytes (excluding null terminator) that this function will write.
// Returns the number of bytes written, EXCLUDING the null terminator.

function stringToUTF8(str, outPtr, maxBytesToWrite) {
  assert(typeof maxBytesToWrite == 'number', 'stringToUTF8(str, outPtr, maxBytesToWrite) is missing the third parameter that specifies the length of the output buffer!');
  return stringToUTF8Array(str, HEAPU8,outPtr, maxBytesToWrite);
}

// Returns the number of bytes the given Javascript string takes if encoded as a UTF8 byte array, EXCLUDING the null terminator byte.
function lengthBytesUTF8(str) {
  var len = 0;
  for (var i = 0; i < str.length; ++i) {
    // Gotcha: charCodeAt returns a 16-bit word that is a UTF-16 encoded code unit, not a Unicode code point of the character! So decode UTF16->UTF32->UTF8.
    // See http://unicode.org/faq/utf_bom.html#utf16-3
    var u = str.charCodeAt(i); // possibly a lead surrogate
    if (u >= 0xD800 && u <= 0xDFFF) u = 0x10000 + ((u & 0x3FF) << 10) | (str.charCodeAt(++i) & 0x3FF);
    if (u <= 0x7F) ++len;
    else if (u <= 0x7FF) len += 2;
    else if (u <= 0xFFFF) len += 3;
    else len += 4;
  }
  return len;
}

// end include: runtime_strings.js
// include: runtime_strings_extra.js


// runtime_strings_extra.js: Strings related runtime functions that are available only in regular runtime.

// Given a pointer 'ptr' to a null-terminated ASCII-encoded string in the emscripten HEAP, returns
// a copy of that string as a Javascript String object.

function AsciiToString(ptr) {
  var str = '';
  while (1) {
    var ch = HEAPU8[((ptr++)>>0)];
    if (!ch) return str;
    str += String.fromCharCode(ch);
  }
}

// Copies the given Javascript String object 'str' to the emscripten HEAP at address 'outPtr',
// null-terminated and encoded in ASCII form. The copy will require at most str.length+1 bytes of space in the HEAP.

function stringToAscii(str, outPtr) {
  return writeAsciiToMemory(str, outPtr, false);
}

// Given a pointer 'ptr' to a null-terminated UTF16LE-encoded string in the emscripten HEAP, returns
// a copy of that string as a Javascript String object.

var UTF16Decoder = typeof TextDecoder != 'undefined' ? new TextDecoder('utf-16le') : undefined;

function UTF16ToString(ptr, maxBytesToRead) {
  assert(ptr % 2 == 0, 'Pointer passed to UTF16ToString must be aligned to two bytes!');
  var endPtr = ptr;
  // TextDecoder needs to know the byte length in advance, it doesn't stop on null terminator by itself.
  // Also, use the length info to avoid running tiny strings through TextDecoder, since .subarray() allocates garbage.
  var idx = endPtr >> 1;
  var maxIdx = idx + maxBytesToRead / 2;
  // If maxBytesToRead is not passed explicitly, it will be undefined, and this
  // will always evaluate to true. This saves on code size.
  while (!(idx >= maxIdx) && HEAPU16[idx]) ++idx;
  endPtr = idx << 1;

  if (endPtr - ptr > 32 && UTF16Decoder) {
    return UTF16Decoder.decode(HEAPU8.subarray(ptr, endPtr));
  } else {
    var str = '';

    // If maxBytesToRead is not passed explicitly, it will be undefined, and the for-loop's condition
    // will always evaluate to true. The loop is then terminated on the first null char.
    for (var i = 0; !(i >= maxBytesToRead / 2); ++i) {
      var codeUnit = HEAP16[(((ptr)+(i*2))>>1)];
      if (codeUnit == 0) break;
      // fromCharCode constructs a character from a UTF-16 code unit, so we can pass the UTF16 string right through.
      str += String.fromCharCode(codeUnit);
    }

    return str;
  }
}

// Copies the given Javascript String object 'str' to the emscripten HEAP at address 'outPtr',
// null-terminated and encoded in UTF16 form. The copy will require at most str.length*4+2 bytes of space in the HEAP.
// Use the function lengthBytesUTF16() to compute the exact number of bytes (excluding null terminator) that this function will write.
// Parameters:
//   str: the Javascript string to copy.
//   outPtr: Byte address in Emscripten HEAP where to write the string to.
//   maxBytesToWrite: The maximum number of bytes this function can write to the array. This count should include the null
//                    terminator, i.e. if maxBytesToWrite=2, only the null terminator will be written and nothing else.
//                    maxBytesToWrite<2 does not write any bytes to the output, not even the null terminator.
// Returns the number of bytes written, EXCLUDING the null terminator.

function stringToUTF16(str, outPtr, maxBytesToWrite) {
  assert(outPtr % 2 == 0, 'Pointer passed to stringToUTF16 must be aligned to two bytes!');
  assert(typeof maxBytesToWrite == 'number', 'stringToUTF16(str, outPtr, maxBytesToWrite) is missing the third parameter that specifies the length of the output buffer!');
  // Backwards compatibility: if max bytes is not specified, assume unsafe unbounded write is allowed.
  if (maxBytesToWrite === undefined) {
    maxBytesToWrite = 0x7FFFFFFF;
  }
  if (maxBytesToWrite < 2) return 0;
  maxBytesToWrite -= 2; // Null terminator.
  var startPtr = outPtr;
  var numCharsToWrite = (maxBytesToWrite < str.length*2) ? (maxBytesToWrite / 2) : str.length;
  for (var i = 0; i < numCharsToWrite; ++i) {
    // charCodeAt returns a UTF-16 encoded code unit, so it can be directly written to the HEAP.
    var codeUnit = str.charCodeAt(i); // possibly a lead surrogate
    HEAP16[((outPtr)>>1)] = codeUnit;
    outPtr += 2;
  }
  // Null-terminate the pointer to the HEAP.
  HEAP16[((outPtr)>>1)] = 0;
  return outPtr - startPtr;
}

// Returns the number of bytes the given Javascript string takes if encoded as a UTF16 byte array, EXCLUDING the null terminator byte.

function lengthBytesUTF16(str) {
  return str.length*2;
}

function UTF32ToString(ptr, maxBytesToRead) {
  assert(ptr % 4 == 0, 'Pointer passed to UTF32ToString must be aligned to four bytes!');
  var i = 0;

  var str = '';
  // If maxBytesToRead is not passed explicitly, it will be undefined, and this
  // will always evaluate to true. This saves on code size.
  while (!(i >= maxBytesToRead / 4)) {
    var utf32 = HEAP32[(((ptr)+(i*4))>>2)];
    if (utf32 == 0) break;
    ++i;
    // Gotcha: fromCharCode constructs a character from a UTF-16 encoded code (pair), not from a Unicode code point! So encode the code point to UTF-16 for constructing.
    // See http://unicode.org/faq/utf_bom.html#utf16-3
    if (utf32 >= 0x10000) {
      var ch = utf32 - 0x10000;
      str += String.fromCharCode(0xD800 | (ch >> 10), 0xDC00 | (ch & 0x3FF));
    } else {
      str += String.fromCharCode(utf32);
    }
  }
  return str;
}

// Copies the given Javascript String object 'str' to the emscripten HEAP at address 'outPtr',
// null-terminated and encoded in UTF32 form. The copy will require at most str.length*4+4 bytes of space in the HEAP.
// Use the function lengthBytesUTF32() to compute the exact number of bytes (excluding null terminator) that this function will write.
// Parameters:
//   str: the Javascript string to copy.
//   outPtr: Byte address in Emscripten HEAP where to write the string to.
//   maxBytesToWrite: The maximum number of bytes this function can write to the array. This count should include the null
//                    terminator, i.e. if maxBytesToWrite=4, only the null terminator will be written and nothing else.
//                    maxBytesToWrite<4 does not write any bytes to the output, not even the null terminator.
// Returns the number of bytes written, EXCLUDING the null terminator.

function stringToUTF32(str, outPtr, maxBytesToWrite) {
  assert(outPtr % 4 == 0, 'Pointer passed to stringToUTF32 must be aligned to four bytes!');
  assert(typeof maxBytesToWrite == 'number', 'stringToUTF32(str, outPtr, maxBytesToWrite) is missing the third parameter that specifies the length of the output buffer!');
  // Backwards compatibility: if max bytes is not specified, assume unsafe unbounded write is allowed.
  if (maxBytesToWrite === undefined) {
    maxBytesToWrite = 0x7FFFFFFF;
  }
  if (maxBytesToWrite < 4) return 0;
  var startPtr = outPtr;
  var endPtr = startPtr + maxBytesToWrite - 4;
  for (var i = 0; i < str.length; ++i) {
    // Gotcha: charCodeAt returns a 16-bit word that is a UTF-16 encoded code unit, not a Unicode code point of the character! We must decode the string to UTF-32 to the heap.
    // See http://unicode.org/faq/utf_bom.html#utf16-3
    var codeUnit = str.charCodeAt(i); // possibly a lead surrogate
    if (codeUnit >= 0xD800 && codeUnit <= 0xDFFF) {
      var trailSurrogate = str.charCodeAt(++i);
      codeUnit = 0x10000 + ((codeUnit & 0x3FF) << 10) | (trailSurrogate & 0x3FF);
    }
    HEAP32[((outPtr)>>2)] = codeUnit;
    outPtr += 4;
    if (outPtr + 4 > endPtr) break;
  }
  // Null-terminate the pointer to the HEAP.
  HEAP32[((outPtr)>>2)] = 0;
  return outPtr - startPtr;
}

// Returns the number of bytes the given Javascript string takes if encoded as a UTF16 byte array, EXCLUDING the null terminator byte.

function lengthBytesUTF32(str) {
  var len = 0;
  for (var i = 0; i < str.length; ++i) {
    // Gotcha: charCodeAt returns a 16-bit word that is a UTF-16 encoded code unit, not a Unicode code point of the character! We must decode the string to UTF-32 to the heap.
    // See http://unicode.org/faq/utf_bom.html#utf16-3
    var codeUnit = str.charCodeAt(i);
    if (codeUnit >= 0xD800 && codeUnit <= 0xDFFF) ++i; // possibly a lead surrogate, so skip over the tail surrogate.
    len += 4;
  }

  return len;
}

// Allocate heap space for a JS string, and write it there.
// It is the responsibility of the caller to free() that memory.
function allocateUTF8(str) {
  var size = lengthBytesUTF8(str) + 1;
  var ret = _malloc(size);
  if (ret) stringToUTF8Array(str, HEAP8, ret, size);
  return ret;
}

// Allocate stack space for a JS string, and write it there.
function allocateUTF8OnStack(str) {
  var size = lengthBytesUTF8(str) + 1;
  var ret = stackAlloc(size);
  stringToUTF8Array(str, HEAP8, ret, size);
  return ret;
}

// Deprecated: This function should not be called because it is unsafe and does not provide
// a maximum length limit of how many bytes it is allowed to write. Prefer calling the
// function stringToUTF8Array() instead, which takes in a maximum length that can be used
// to be secure from out of bounds writes.
/** @deprecated
    @param {boolean=} dontAddNull */
function writeStringToMemory(string, buffer, dontAddNull) {
  warnOnce('writeStringToMemory is deprecated and should not be called! Use stringToUTF8() instead!');

  var /** @type {number} */ lastChar, /** @type {number} */ end;
  if (dontAddNull) {
    // stringToUTF8Array always appends null. If we don't want to do that, remember the
    // character that existed at the location where the null will be placed, and restore
    // that after the write (below).
    end = buffer + lengthBytesUTF8(string);
    lastChar = HEAP8[end];
  }
  stringToUTF8(string, buffer, Infinity);
  if (dontAddNull) HEAP8[end] = lastChar; // Restore the value under the null character.
}

function writeArrayToMemory(array, buffer) {
  assert(array.length >= 0, 'writeArrayToMemory array must have a length (should be an array or typed array)')
  HEAP8.set(array, buffer);
}

/** @param {boolean=} dontAddNull */
function writeAsciiToMemory(str, buffer, dontAddNull) {
  for (var i = 0; i < str.length; ++i) {
    assert(str.charCodeAt(i) === (str.charCodeAt(i) & 0xff));
    HEAP8[((buffer++)>>0)] = str.charCodeAt(i);
  }
  // Null-terminate the pointer to the HEAP.
  if (!dontAddNull) HEAP8[((buffer)>>0)] = 0;
}

// end include: runtime_strings_extra.js
// Memory management

var HEAP,
/** @type {!ArrayBuffer} */
  buffer,
/** @type {!Int8Array} */
  HEAP8,
/** @type {!Uint8Array} */
  HEAPU8,
/** @type {!Int16Array} */
  HEAP16,
/** @type {!Uint16Array} */
  HEAPU16,
/** @type {!Int32Array} */
  HEAP32,
/** @type {!Uint32Array} */
  HEAPU32,
/** @type {!Float32Array} */
  HEAPF32,
/** @type {!Float64Array} */
  HEAPF64;

function updateGlobalBufferAndViews(buf) {
  buffer = buf;
  Module['HEAP8'] = HEAP8 = new Int8Array(buf);
  Module['HEAP16'] = HEAP16 = new Int16Array(buf);
  Module['HEAP32'] = HEAP32 = new Int32Array(buf);
  Module['HEAPU8'] = HEAPU8 = new Uint8Array(buf);
  Module['HEAPU16'] = HEAPU16 = new Uint16Array(buf);
  Module['HEAPU32'] = HEAPU32 = new Uint32Array(buf);
  Module['HEAPF32'] = HEAPF32 = new Float32Array(buf);
  Module['HEAPF64'] = HEAPF64 = new Float64Array(buf);
}

var TOTAL_STACK = 5242880;
if (Module['TOTAL_STACK']) assert(TOTAL_STACK === Module['TOTAL_STACK'], 'the stack size can no longer be determined at runtime')

var INITIAL_MEMORY = Module['INITIAL_MEMORY'] || 16777216;legacyModuleProp('INITIAL_MEMORY', 'INITIAL_MEMORY');

assert(INITIAL_MEMORY >= TOTAL_STACK, 'INITIAL_MEMORY should be larger than TOTAL_STACK, was ' + INITIAL_MEMORY + '! (TOTAL_STACK=' + TOTAL_STACK + ')');

// check for full engine support (use string 'subarray' to avoid closure compiler confusion)
assert(typeof Int32Array != 'undefined' && typeof Float64Array !== 'undefined' && Int32Array.prototype.subarray != undefined && Int32Array.prototype.set != undefined,
       'JS engine does not provide full typed array support');

// If memory is defined in wasm, the user can't provide it.
assert(!Module['wasmMemory'], 'Use of `wasmMemory` detected.  Use -sIMPORTED_MEMORY to define wasmMemory externally');
assert(INITIAL_MEMORY == 16777216, 'Detected runtime INITIAL_MEMORY setting.  Use -sIMPORTED_MEMORY to define wasmMemory dynamically');

// include: runtime_init_table.js
// In regular non-RELOCATABLE mode the table is exported
// from the wasm module and this will be assigned once
// the exports are available.
var wasmTable;

// end include: runtime_init_table.js
// include: runtime_stack_check.js


// Initializes the stack cookie. Called at the startup of main and at the startup of each thread in pthreads mode.
function writeStackCookie() {
  var max = _emscripten_stack_get_end();
  assert((max & 3) == 0);
  // The stack grow downwards towards _emscripten_stack_get_end.
  // We write cookies to the final two words in the stack and detect if they are
  // ever overwritten.
  HEAP32[((max)>>2)] = 0x2135467;
  HEAP32[(((max)+(4))>>2)] = 0x89BACDFE;
  // Also test the global address 0 for integrity.
  HEAPU32[0] = 0x63736d65; /* 'emsc' */
}

function checkStackCookie() {
  if (ABORT) return;
  var max = _emscripten_stack_get_end();
  var cookie1 = HEAPU32[((max)>>2)];
  var cookie2 = HEAPU32[(((max)+(4))>>2)];
  if (cookie1 != 0x2135467 || cookie2 != 0x89BACDFE) {
    abort('Stack overflow! Stack cookie has been overwritten, expected hex dwords 0x89BACDFE and 0x2135467, but received 0x' + cookie2.toString(16) + ' 0x' + cookie1.toString(16));
  }
  // Also test the global address 0 for integrity.
  if (HEAPU32[0] !== 0x63736d65 /* 'emsc' */) abort('Runtime error: The application has corrupted its heap memory area (address zero)!');
}

// end include: runtime_stack_check.js
// include: runtime_assertions.js


// Endianness check
(function() {
  var h16 = new Int16Array(1);
  var h8 = new Int8Array(h16.buffer);
  h16[0] = 0x6373;
  if (h8[0] !== 0x73 || h8[1] !== 0x63) throw 'Runtime error: expected the system to be little-endian! (Run with -sSUPPORT_BIG_ENDIAN to bypass)';
})();

// end include: runtime_assertions.js
var __ATPRERUN__  = []; // functions called before the runtime is initialized
var __ATINIT__    = []; // functions called during startup
var __ATEXIT__    = []; // functions called during shutdown
var __ATPOSTRUN__ = []; // functions called after the main() is called

var runtimeInitialized = false;

function keepRuntimeAlive() {
  return noExitRuntime;
}

function preRun() {

  if (Module['preRun']) {
    if (typeof Module['preRun'] == 'function') Module['preRun'] = [Module['preRun']];
    while (Module['preRun'].length) {
      addOnPreRun(Module['preRun'].shift());
    }
  }

  callRuntimeCallbacks(__ATPRERUN__);
}

function initRuntime() {
  checkStackCookie();
  assert(!runtimeInitialized);
  runtimeInitialized = true;

  
  callRuntimeCallbacks(__ATINIT__);
}

function postRun() {
  checkStackCookie();

  if (Module['postRun']) {
    if (typeof Module['postRun'] == 'function') Module['postRun'] = [Module['postRun']];
    while (Module['postRun'].length) {
      addOnPostRun(Module['postRun'].shift());
    }
  }

  callRuntimeCallbacks(__ATPOSTRUN__);
}

function addOnPreRun(cb) {
  __ATPRERUN__.unshift(cb);
}

function addOnInit(cb) {
  __ATINIT__.unshift(cb);
}

function addOnExit(cb) {
}

function addOnPostRun(cb) {
  __ATPOSTRUN__.unshift(cb);
}

// include: runtime_math.js


// https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/imul

// https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/fround

// https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/clz32

// https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/trunc

assert(Math.imul, 'This browser does not support Math.imul(), build with LEGACY_VM_SUPPORT or POLYFILL_OLD_MATH_FUNCTIONS to add in a polyfill');
assert(Math.fround, 'This browser does not support Math.fround(), build with LEGACY_VM_SUPPORT or POLYFILL_OLD_MATH_FUNCTIONS to add in a polyfill');
assert(Math.clz32, 'This browser does not support Math.clz32(), build with LEGACY_VM_SUPPORT or POLYFILL_OLD_MATH_FUNCTIONS to add in a polyfill');
assert(Math.trunc, 'This browser does not support Math.trunc(), build with LEGACY_VM_SUPPORT or POLYFILL_OLD_MATH_FUNCTIONS to add in a polyfill');

// end include: runtime_math.js
// A counter of dependencies for calling run(). If we need to
// do asynchronous work before running, increment this and
// decrement it. Incrementing must happen in a place like
// Module.preRun (used by emcc to add file preloading).
// Note that you can add dependencies in preRun, even though
// it happens right before run - run will be postponed until
// the dependencies are met.
var runDependencies = 0;
var runDependencyWatcher = null;
var dependenciesFulfilled = null; // overridden to take different actions when all run dependencies are fulfilled
var runDependencyTracking = {};

function getUniqueRunDependency(id) {
  var orig = id;
  while (1) {
    if (!runDependencyTracking[id]) return id;
    id = orig + Math.random();
  }
}

function addRunDependency(id) {
  runDependencies++;

  if (Module['monitorRunDependencies']) {
    Module['monitorRunDependencies'](runDependencies);
  }

  if (id) {
    assert(!runDependencyTracking[id]);
    runDependencyTracking[id] = 1;
    if (runDependencyWatcher === null && typeof setInterval != 'undefined') {
      // Check for missing dependencies every few seconds
      runDependencyWatcher = setInterval(function() {
        if (ABORT) {
          clearInterval(runDependencyWatcher);
          runDependencyWatcher = null;
          return;
        }
        var shown = false;
        for (var dep in runDependencyTracking) {
          if (!shown) {
            shown = true;
            err('still waiting on run dependencies:');
          }
          err('dependency: ' + dep);
        }
        if (shown) {
          err('(end of list)');
        }
      }, 10000);
    }
  } else {
    err('warning: run dependency added without ID');
  }
}

function removeRunDependency(id) {
  runDependencies--;

  if (Module['monitorRunDependencies']) {
    Module['monitorRunDependencies'](runDependencies);
  }

  if (id) {
    assert(runDependencyTracking[id]);
    delete runDependencyTracking[id];
  } else {
    err('warning: run dependency removed without ID');
  }
  if (runDependencies == 0) {
    if (runDependencyWatcher !== null) {
      clearInterval(runDependencyWatcher);
      runDependencyWatcher = null;
    }
    if (dependenciesFulfilled) {
      var callback = dependenciesFulfilled;
      dependenciesFulfilled = null;
      callback(); // can add another dependenciesFulfilled
    }
  }
}

/** @param {string|number=} what */
function abort(what) {
  {
    if (Module['onAbort']) {
      Module['onAbort'](what);
    }
  }

  what = 'Aborted(' + what + ')';
  // TODO(sbc): Should we remove printing and leave it up to whoever
  // catches the exception?
  err(what);

  ABORT = true;
  EXITSTATUS = 1;

  // Use a wasm runtime error, because a JS error might be seen as a foreign
  // exception, which means we'd run destructors on it. We need the error to
  // simply make the program stop.
  // FIXME This approach does not work in Wasm EH because it currently does not assume
  // all RuntimeErrors are from traps; it decides whether a RuntimeError is from
  // a trap or not based on a hidden field within the object. So at the moment
  // we don't have a way of throwing a wasm trap from JS. TODO Make a JS API that
  // allows this in the wasm spec.

  // Suppress closure compiler warning here. Closure compiler's builtin extern
  // defintion for WebAssembly.RuntimeError claims it takes no arguments even
  // though it can.
  // TODO(https://github.com/google/closure-compiler/pull/3913): Remove if/when upstream closure gets fixed.
  /** @suppress {checkTypes} */
  var e = new WebAssembly.RuntimeError(what);

  readyPromiseReject(e);
  // Throw the error whether or not MODULARIZE is set because abort is used
  // in code paths apart from instantiation where an exception is expected
  // to be thrown when abort is called.
  throw e;
}

// {{MEM_INITIALIZER}}

// include: memoryprofiler.js


// end include: memoryprofiler.js
// show errors on likely calls to FS when it was not included
var FS = {
  error: function() {
    abort('Filesystem support (FS) was not included. The problem is that you are using files from JS, but files were not used from C/C++, so filesystem support was not auto-included. You can force-include filesystem support with -sFORCE_FILESYSTEM');
  },
  init: function() { FS.error() },
  createDataFile: function() { FS.error() },
  createPreloadedFile: function() { FS.error() },
  createLazyFile: function() { FS.error() },
  open: function() { FS.error() },
  mkdev: function() { FS.error() },
  registerDevice: function() { FS.error() },
  analyzePath: function() { FS.error() },
  loadFilesFromDB: function() { FS.error() },

  ErrnoError: function ErrnoError() { FS.error() },
};
Module['FS_createDataFile'] = FS.createDataFile;
Module['FS_createPreloadedFile'] = FS.createPreloadedFile;

// include: URIUtils.js


// Prefix of data URIs emitted by SINGLE_FILE and related options.
var dataURIPrefix = 'data:application/octet-stream;base64,';

// Indicates whether filename is a base64 data URI.
function isDataURI(filename) {
  // Prefix of data URIs emitted by SINGLE_FILE and related options.
  return filename.startsWith(dataURIPrefix);
}

// Indicates whether filename is delivered via file protocol (as opposed to http/https)
function isFileURI(filename) {
  return filename.startsWith('file://');
}

// end include: URIUtils.js
/** @param {boolean=} fixedasm */
function createExportWrapper(name, fixedasm) {
  return function() {
    var displayName = name;
    var asm = fixedasm;
    if (!fixedasm) {
      asm = Module['asm'];
    }
    assert(runtimeInitialized, 'native function `' + displayName + '` called before runtime initialization');
    if (!asm[name]) {
      assert(asm[name], 'exported native function `' + displayName + '` not found');
    }
    return asm[name].apply(null, arguments);
  };
}

var wasmBinaryFile;
  wasmBinaryFile = 'ephys.wasm';
  if (!isDataURI(wasmBinaryFile)) {
    wasmBinaryFile = locateFile(wasmBinaryFile);
  }

function getBinary(file) {
  try {
    if (file == wasmBinaryFile && wasmBinary) {
      return new Uint8Array(wasmBinary);
    }
    if (readBinary) {
      return readBinary(file);
    } else {
      throw "both async and sync fetching of the wasm failed";
    }
  }
  catch (err) {
    abort(err);
  }
}

function getBinaryPromise() {
  // If we don't have the binary yet, try to to load it asynchronously.
  // Fetch has some additional restrictions over XHR, like it can't be used on a file:// url.
  // See https://github.com/github/fetch/pull/92#issuecomment-140665932
  // Cordova or Electron apps are typically loaded from a file:// url.
  // So use fetch if it is available and the url is not a file, otherwise fall back to XHR.
  if (!wasmBinary && (ENVIRONMENT_IS_WEB || ENVIRONMENT_IS_WORKER)) {
    if (typeof fetch == 'function'
      && !isFileURI(wasmBinaryFile)
    ) {
      return fetch(wasmBinaryFile, { credentials: 'same-origin' }).then(function(response) {
        if (!response['ok']) {
          throw "failed to load wasm binary file at '" + wasmBinaryFile + "'";
        }
        return response['arrayBuffer']();
      }).catch(function () {
          return getBinary(wasmBinaryFile);
      });
    }
    else {
      if (readAsync) {
        // fetch is not available or url is file => try XHR (readAsync uses XHR internally)
        return new Promise(function(resolve, reject) {
          readAsync(wasmBinaryFile, function(response) { resolve(new Uint8Array(/** @type{!ArrayBuffer} */(response))) }, reject)
        });
      }
    }
  }

  // Otherwise, getBinary should be able to get it synchronously
  return Promise.resolve().then(function() { return getBinary(wasmBinaryFile); });
}

// Create the wasm instance.
// Receives the wasm imports, returns the exports.
function createWasm() {
  // prepare imports
  var info = {
    'env': asmLibraryArg,
    'wasi_snapshot_preview1': asmLibraryArg,
  };
  // Load the wasm module and create an instance of using native support in the JS engine.
  // handle a generated wasm instance, receiving its exports and
  // performing other necessary setup
  /** @param {WebAssembly.Module=} module*/
  function receiveInstance(instance, module) {
    var exports = instance.exports;

    Module['asm'] = exports;

    wasmMemory = Module['asm']['memory'];
    assert(wasmMemory, "memory not found in wasm exports");
    // This assertion doesn't hold when emscripten is run in --post-link
    // mode.
    // TODO(sbc): Read INITIAL_MEMORY out of the wasm file in post-link mode.
    //assert(wasmMemory.buffer.byteLength === 16777216);
    updateGlobalBufferAndViews(wasmMemory.buffer);

    wasmTable = Module['asm']['__indirect_function_table'];
    assert(wasmTable, "table not found in wasm exports");

    addOnInit(Module['asm']['__wasm_call_ctors']);

    removeRunDependency('wasm-instantiate');

  }
  // we can't run yet (except in a pthread, where we have a custom sync instantiator)
  addRunDependency('wasm-instantiate');

  // Prefer streaming instantiation if available.
  // Async compilation can be confusing when an error on the page overwrites Module
  // (for example, if the order of elements is wrong, and the one defining Module is
  // later), so we save Module and check it later.
  var trueModule = Module;
  function receiveInstantiationResult(result) {
    // 'result' is a ResultObject object which has both the module and instance.
    // receiveInstance() will swap in the exports (to Module.asm) so they can be called
    assert(Module === trueModule, 'the Module object should not be replaced during async compilation - perhaps the order of HTML elements is wrong?');
    trueModule = null;
    // TODO: Due to Closure regression https://github.com/google/closure-compiler/issues/3193, the above line no longer optimizes out down to the following line.
    // When the regression is fixed, can restore the above USE_PTHREADS-enabled path.
    receiveInstance(result['instance']);
  }

  function instantiateArrayBuffer(receiver) {
    return getBinaryPromise().then(function(binary) {
      return WebAssembly.instantiate(binary, info);
    }).then(function (instance) {
      return instance;
    }).then(receiver, function(reason) {
      err('failed to asynchronously prepare wasm: ' + reason);

      // Warn on some common problems.
      if (isFileURI(wasmBinaryFile)) {
        err('warning: Loading from a file URI (' + wasmBinaryFile + ') is not supported in most browsers. See https://emscripten.org/docs/getting_started/FAQ.html#how-do-i-run-a-local-webserver-for-testing-why-does-my-program-stall-in-downloading-or-preparing');
      }
      abort(reason);
    });
  }

  function instantiateAsync() {
    if (!wasmBinary &&
        typeof WebAssembly.instantiateStreaming == 'function' &&
        !isDataURI(wasmBinaryFile) &&
        // Don't use streaming for file:// delivered objects in a webview, fetch them synchronously.
        !isFileURI(wasmBinaryFile) &&
        typeof fetch == 'function') {
      return fetch(wasmBinaryFile, { credentials: 'same-origin' }).then(function(response) {
        // Suppress closure warning here since the upstream definition for
        // instantiateStreaming only allows Promise<Repsponse> rather than
        // an actual Response.
        // TODO(https://github.com/google/closure-compiler/pull/3913): Remove if/when upstream closure is fixed.
        /** @suppress {checkTypes} */
        var result = WebAssembly.instantiateStreaming(response, info);

        return result.then(
          receiveInstantiationResult,
          function(reason) {
            // We expect the most common failure cause to be a bad MIME type for the binary,
            // in which case falling back to ArrayBuffer instantiation should work.
            err('wasm streaming compile failed: ' + reason);
            err('falling back to ArrayBuffer instantiation');
            return instantiateArrayBuffer(receiveInstantiationResult);
          });
      });
    } else {
      return instantiateArrayBuffer(receiveInstantiationResult);
    }
  }

  // User shell pages can write their own Module.instantiateWasm = function(imports, successCallback) callback
  // to manually instantiate the Wasm module themselves. This allows pages to run the instantiation parallel
  // to any other async startup actions they are performing.
  // Also pthreads and wasm workers initialize the wasm instance through this path.
  if (Module['instantiateWasm']) {
    try {
      var exports = Module['instantiateWasm'](info, receiveInstance);
      return exports;
    } catch(e) {
      err('Module.instantiateWasm callback failed with error: ' + e);
      return false;
    }
  }

  // If instantiation fails, reject the module ready promise.
  instantiateAsync().catch(readyPromiseReject);
  return {}; // no exports yet; we'll fill them in later
}

// Globals used by JS i64 conversions (see makeSetValue)
var tempDouble;
var tempI64;

// === Body ===

var ASM_CONSTS = {
  
};
function array_bounds_check_error(idx,size) { throw 'Array index ' + idx + ' out of bounds: [0,' + size + ')'; }





  function callRuntimeCallbacks(callbacks) {
      while (callbacks.length > 0) {
        var callback = callbacks.shift();
        if (typeof callback == 'function') {
          callback(Module); // Pass the module as the first argument.
          continue;
        }
        var func = callback.func;
        if (typeof func == 'number') {
          if (callback.arg === undefined) {
            // Run the wasm function ptr with signature 'v'. If no function
            // with such signature was exported, this call does not need
            // to be emitted (and would confuse Closure)
            getWasmTableEntry(func)();
          } else {
            // If any function with signature 'vi' was exported, run
            // the callback with that signature.
            getWasmTableEntry(func)(callback.arg);
          }
        } else {
          func(callback.arg === undefined ? null : callback.arg);
        }
      }
    }

  function withStackSave(f) {
      var stack = stackSave();
      var ret = f();
      stackRestore(stack);
      return ret;
    }
  function demangle(func) {
      warnOnce('warning: build with -sDEMANGLE_SUPPORT to link in libcxxabi demangling');
      return func;
    }

  function demangleAll(text) {
      var regex =
        /\b_Z[\w\d_]+/g;
      return text.replace(regex,
        function(x) {
          var y = demangle(x);
          return x === y ? x : (y + ' [' + x + ']');
        });
    }

  
    /**
     * @param {number} ptr
     * @param {string} type
     */
  function getValue(ptr, type = 'i8') {
      if (type.endsWith('*')) type = 'i32';
      switch (type) {
        case 'i1': return HEAP8[((ptr)>>0)];
        case 'i8': return HEAP8[((ptr)>>0)];
        case 'i16': return HEAP16[((ptr)>>1)];
        case 'i32': return HEAP32[((ptr)>>2)];
        case 'i64': return HEAP32[((ptr)>>2)];
        case 'float': return HEAPF32[((ptr)>>2)];
        case 'double': return Number(HEAPF64[((ptr)>>3)]);
        default: abort('invalid type for getValue: ' + type);
      }
      return null;
    }

  var wasmTableMirror = [];
  function getWasmTableEntry(funcPtr) {
      var func = wasmTableMirror[funcPtr];
      if (!func) {
        if (funcPtr >= wasmTableMirror.length) wasmTableMirror.length = funcPtr + 1;
        wasmTableMirror[funcPtr] = func = wasmTable.get(funcPtr);
      }
      assert(wasmTable.get(funcPtr) == func, "JavaScript-side Wasm function table mirror is out of date!");
      return func;
    }

  function handleException(e) {
      // Certain exception types we do not treat as errors since they are used for
      // internal control flow.
      // 1. ExitStatus, which is thrown by exit()
      // 2. "unwind", which is thrown by emscripten_unwind_to_js_event_loop() and others
      //    that wish to return to JS event loop.
      if (e instanceof ExitStatus || e == 'unwind') {
        return EXITSTATUS;
      }
      quit_(1, e);
    }

  function jsStackTrace() {
      var error = new Error();
      if (!error.stack) {
        // IE10+ special cases: It does have callstack info, but it is only
        // populated if an Error object is thrown, so try that as a special-case.
        try {
          throw new Error();
        } catch(e) {
          error = e;
        }
        if (!error.stack) {
          return '(no stack trace available)';
        }
      }
      return error.stack.toString();
    }

  
    /**
     * @param {number} ptr
     * @param {number} value
     * @param {string} type
     */
  function setValue(ptr, value, type = 'i8') {
      if (type.endsWith('*')) type = 'i32';
      switch (type) {
        case 'i1': HEAP8[((ptr)>>0)] = value; break;
        case 'i8': HEAP8[((ptr)>>0)] = value; break;
        case 'i16': HEAP16[((ptr)>>1)] = value; break;
        case 'i32': HEAP32[((ptr)>>2)] = value; break;
        case 'i64': (tempI64 = [value>>>0,(tempDouble=value,(+(Math.abs(tempDouble))) >= 1.0 ? (tempDouble > 0.0 ? ((Math.min((+(Math.floor((tempDouble)/4294967296.0))), 4294967295.0))|0)>>>0 : (~~((+(Math.ceil((tempDouble - +(((~~(tempDouble)))>>>0))/4294967296.0)))))>>>0) : 0)],HEAP32[((ptr)>>2)] = tempI64[0],HEAP32[(((ptr)+(4))>>2)] = tempI64[1]); break;
        case 'float': HEAPF32[((ptr)>>2)] = value; break;
        case 'double': HEAPF64[((ptr)>>3)] = value; break;
        default: abort('invalid type for setValue: ' + type);
      }
    }

  function setWasmTableEntry(idx, func) {
      wasmTable.set(idx, func);
      // With ABORT_ON_WASM_EXCEPTIONS wasmTable.get is overriden to return wrapped
      // functions so we need to call it here to retrieve the potential wrapper correctly
      // instead of just storing 'func' directly into wasmTableMirror
      wasmTableMirror[idx] = wasmTable.get(idx);
    }

  function stackTrace() {
      var js = jsStackTrace();
      if (Module['extraStackTrace']) js += '\n' + Module['extraStackTrace']();
      return demangleAll(js);
    }

  function ___assert_fail(condition, filename, line, func) {
      abort('Assertion failed: ' + UTF8ToString(condition) + ', at: ' + [filename ? UTF8ToString(filename) : 'unknown filename', line, func ? UTF8ToString(func) : 'unknown function']);
    }

  function ___cxa_allocate_exception(size) {
      // Thrown object is prepended by exception metadata block
      return _malloc(size + 24) + 24;
    }

  /** @constructor */
  function ExceptionInfo(excPtr) {
      this.excPtr = excPtr;
      this.ptr = excPtr - 24;
  
      this.set_type = function(type) {
        HEAPU32[(((this.ptr)+(4))>>2)] = type;
      };
  
      this.get_type = function() {
        return HEAPU32[(((this.ptr)+(4))>>2)];
      };
  
      this.set_destructor = function(destructor) {
        HEAPU32[(((this.ptr)+(8))>>2)] = destructor;
      };
  
      this.get_destructor = function() {
        return HEAPU32[(((this.ptr)+(8))>>2)];
      };
  
      this.set_refcount = function(refcount) {
        HEAP32[((this.ptr)>>2)] = refcount;
      };
  
      this.set_caught = function (caught) {
        caught = caught ? 1 : 0;
        HEAP8[(((this.ptr)+(12))>>0)] = caught;
      };
  
      this.get_caught = function () {
        return HEAP8[(((this.ptr)+(12))>>0)] != 0;
      };
  
      this.set_rethrown = function (rethrown) {
        rethrown = rethrown ? 1 : 0;
        HEAP8[(((this.ptr)+(13))>>0)] = rethrown;
      };
  
      this.get_rethrown = function () {
        return HEAP8[(((this.ptr)+(13))>>0)] != 0;
      };
  
      // Initialize native structure fields. Should be called once after allocated.
      this.init = function(type, destructor) {
        this.set_adjusted_ptr(0);
        this.set_type(type);
        this.set_destructor(destructor);
        this.set_refcount(0);
        this.set_caught(false);
        this.set_rethrown(false);
      }
  
      this.add_ref = function() {
        var value = HEAP32[((this.ptr)>>2)];
        HEAP32[((this.ptr)>>2)] = value + 1;
      };
  
      // Returns true if last reference released.
      this.release_ref = function() {
        var prev = HEAP32[((this.ptr)>>2)];
        HEAP32[((this.ptr)>>2)] = prev - 1;
        assert(prev > 0);
        return prev === 1;
      };
  
      this.set_adjusted_ptr = function(adjustedPtr) {
        HEAPU32[(((this.ptr)+(16))>>2)] = adjustedPtr;
      };
  
      this.get_adjusted_ptr = function() {
        return HEAPU32[(((this.ptr)+(16))>>2)];
      };
  
      // Get pointer which is expected to be received by catch clause in C++ code. It may be adjusted
      // when the pointer is casted to some of the exception object base classes (e.g. when virtual
      // inheritance is used). When a pointer is thrown this method should return the thrown pointer
      // itself.
      this.get_exception_ptr = function() {
        // Work around a fastcomp bug, this code is still included for some reason in a build without
        // exceptions support.
        var isPointer = ___cxa_is_pointer_type(this.get_type());
        if (isPointer) {
          return HEAPU32[((this.excPtr)>>2)];
        }
        var adjusted = this.get_adjusted_ptr();
        if (adjusted !== 0) return adjusted;
        return this.excPtr;
      };
    }
  
  var exceptionLast = 0;
  
  var uncaughtExceptionCount = 0;
  function ___cxa_throw(ptr, type, destructor) {
      var info = new ExceptionInfo(ptr);
      // Initialize ExceptionInfo content after it was allocated in __cxa_allocate_exception.
      info.init(type, destructor);
      exceptionLast = ptr;
      uncaughtExceptionCount++;
      throw ptr + " - Exception catching is disabled, this exception cannot be caught. Compile with -sNO_DISABLE_EXCEPTION_CATCHING or -sEXCEPTION_CATCHING_ALLOWED=[..] to catch.";
    }

  function _abort() {
      abort('native code called abort()');
    }

  function getHeapMax() {
      return HEAPU8.length;
    }
  
  function abortOnCannotGrowMemory(requestedSize) {
      abort('Cannot enlarge memory arrays to size ' + requestedSize + ' bytes (OOM). Either (1) compile with -sINITIAL_MEMORY=X with X higher than the current value ' + HEAP8.length + ', (2) compile with -sALLOW_MEMORY_GROWTH which allows increasing the size at runtime, or (3) if you want malloc to return NULL (0) instead of this abort, compile with -sABORTING_MALLOC=0');
    }
  function _emscripten_resize_heap(requestedSize) {
      var oldSize = HEAPU8.length;
      requestedSize = requestedSize >>> 0;
      abortOnCannotGrowMemory(requestedSize);
    }
var ASSERTIONS = true;



/** @type {function(string, boolean=, number=)} */
function intArrayFromString(stringy, dontAddNull, length) {
  var len = length > 0 ? length : lengthBytesUTF8(stringy)+1;
  var u8array = new Array(len);
  var numBytesWritten = stringToUTF8Array(stringy, u8array, 0, u8array.length);
  if (dontAddNull) u8array.length = numBytesWritten;
  return u8array;
}

function intArrayToString(array) {
  var ret = [];
  for (var i = 0; i < array.length; i++) {
    var chr = array[i];
    if (chr > 0xFF) {
      if (ASSERTIONS) {
        assert(false, 'Character code ' + chr + ' (' + String.fromCharCode(chr) + ')  at offset ' + i + ' not in 0x00-0xFF.');
      }
      chr &= 0xFF;
    }
    ret.push(String.fromCharCode(chr));
  }
  return ret.join('');
}


function checkIncomingModuleAPI() {
  ignoredModuleProp('fetchSettings');
}
var asmLibraryArg = {
  "__assert_fail": ___assert_fail,
  "__cxa_allocate_exception": ___cxa_allocate_exception,
  "__cxa_throw": ___cxa_throw,
  "abort": _abort,
  "array_bounds_check_error": array_bounds_check_error,
  "emscripten_resize_heap": _emscripten_resize_heap
};
var asm = createWasm();
/** @type {function(...*):?} */
var ___wasm_call_ctors = Module["___wasm_call_ctors"] = createExportWrapper("__wasm_call_ctors");

/** @type {function(...*):?} */
var _emscripten_bind_VoidPtr___destroy___0 = Module["_emscripten_bind_VoidPtr___destroy___0"] = createExportWrapper("emscripten_bind_VoidPtr___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_Vec2_0 = Module["_emscripten_bind_Vec2_Vec2_0"] = createExportWrapper("emscripten_bind_Vec2_Vec2_0");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_Vec2_1 = Module["_emscripten_bind_Vec2_Vec2_1"] = createExportWrapper("emscripten_bind_Vec2_Vec2_1");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_Vec2_2 = Module["_emscripten_bind_Vec2_Vec2_2"] = createExportWrapper("emscripten_bind_Vec2_Vec2_2");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_set_2 = Module["_emscripten_bind_Vec2_set_2"] = createExportWrapper("emscripten_bind_Vec2_set_2");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_norm_0 = Module["_emscripten_bind_Vec2_norm_0"] = createExportWrapper("emscripten_bind_Vec2_norm_0");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_norm2_0 = Module["_emscripten_bind_Vec2_norm2_0"] = createExportWrapper("emscripten_bind_Vec2_norm2_0");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_normalize_0 = Module["_emscripten_bind_Vec2_normalize_0"] = createExportWrapper("emscripten_bind_Vec2_normalize_0");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_add_1 = Module["_emscripten_bind_Vec2_add_1"] = createExportWrapper("emscripten_bind_Vec2_add_1");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_sub_1 = Module["_emscripten_bind_Vec2_sub_1"] = createExportWrapper("emscripten_bind_Vec2_sub_1");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_mult_1 = Module["_emscripten_bind_Vec2_mult_1"] = createExportWrapper("emscripten_bind_Vec2_mult_1");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_get_x_0 = Module["_emscripten_bind_Vec2_get_x_0"] = createExportWrapper("emscripten_bind_Vec2_get_x_0");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_set_x_1 = Module["_emscripten_bind_Vec2_set_x_1"] = createExportWrapper("emscripten_bind_Vec2_set_x_1");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_get_y_0 = Module["_emscripten_bind_Vec2_get_y_0"] = createExportWrapper("emscripten_bind_Vec2_get_y_0");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2_set_y_1 = Module["_emscripten_bind_Vec2_set_y_1"] = createExportWrapper("emscripten_bind_Vec2_set_y_1");

/** @type {function(...*):?} */
var _emscripten_bind_Vec2___destroy___0 = Module["_emscripten_bind_Vec2___destroy___0"] = createExportWrapper("emscripten_bind_Vec2___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_Mat2_0 = Module["_emscripten_bind_Mat2_Mat2_0"] = createExportWrapper("emscripten_bind_Mat2_Mat2_0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_Mat2_1 = Module["_emscripten_bind_Mat2_Mat2_1"] = createExportWrapper("emscripten_bind_Mat2_Mat2_1");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_Mat2_4 = Module["_emscripten_bind_Mat2_Mat2_4"] = createExportWrapper("emscripten_bind_Mat2_Mat2_4");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_zero_0 = Module["_emscripten_bind_Mat2_zero_0"] = createExportWrapper("emscripten_bind_Mat2_zero_0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_identity_0 = Module["_emscripten_bind_Mat2_identity_0"] = createExportWrapper("emscripten_bind_Mat2_identity_0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_at_2 = Module["_emscripten_bind_Mat2_at_2"] = createExportWrapper("emscripten_bind_Mat2_at_2");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_set_3 = Module["_emscripten_bind_Mat2_set_3"] = createExportWrapper("emscripten_bind_Mat2_set_3");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_add_1 = Module["_emscripten_bind_Mat2_add_1"] = createExportWrapper("emscripten_bind_Mat2_add_1");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_sub_1 = Module["_emscripten_bind_Mat2_sub_1"] = createExportWrapper("emscripten_bind_Mat2_sub_1");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_mult_1 = Module["_emscripten_bind_Mat2_mult_1"] = createExportWrapper("emscripten_bind_Mat2_mult_1");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_determinant_0 = Module["_emscripten_bind_Mat2_determinant_0"] = createExportWrapper("emscripten_bind_Mat2_determinant_0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_inverse_0 = Module["_emscripten_bind_Mat2_inverse_0"] = createExportWrapper("emscripten_bind_Mat2_inverse_0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_invert_0 = Module["_emscripten_bind_Mat2_invert_0"] = createExportWrapper("emscripten_bind_Mat2_invert_0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_transpose_0 = Module["_emscripten_bind_Mat2_transpose_0"] = createExportWrapper("emscripten_bind_Mat2_transpose_0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_get_data_1 = Module["_emscripten_bind_Mat2_get_data_1"] = createExportWrapper("emscripten_bind_Mat2_get_data_1");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2_set_data_2 = Module["_emscripten_bind_Mat2_set_data_2"] = createExportWrapper("emscripten_bind_Mat2_set_data_2");

/** @type {function(...*):?} */
var _emscripten_bind_Mat2___destroy___0 = Module["_emscripten_bind_Mat2___destroy___0"] = createExportWrapper("emscripten_bind_Mat2___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat3_Mat3_0 = Module["_emscripten_bind_Mat3_Mat3_0"] = createExportWrapper("emscripten_bind_Mat3_Mat3_0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat3_Mat3_1 = Module["_emscripten_bind_Mat3_Mat3_1"] = createExportWrapper("emscripten_bind_Mat3_Mat3_1");

/** @type {function(...*):?} */
var _emscripten_bind_Mat3_Mat3_6 = Module["_emscripten_bind_Mat3_Mat3_6"] = createExportWrapper("emscripten_bind_Mat3_Mat3_6");

/** @type {function(...*):?} */
var _emscripten_bind_Mat3_identity_0 = Module["_emscripten_bind_Mat3_identity_0"] = createExportWrapper("emscripten_bind_Mat3_identity_0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat3_at_2 = Module["_emscripten_bind_Mat3_at_2"] = createExportWrapper("emscripten_bind_Mat3_at_2");

/** @type {function(...*):?} */
var _emscripten_bind_Mat3_set_3 = Module["_emscripten_bind_Mat3_set_3"] = createExportWrapper("emscripten_bind_Mat3_set_3");

/** @type {function(...*):?} */
var _emscripten_bind_Mat3_determinant_0 = Module["_emscripten_bind_Mat3_determinant_0"] = createExportWrapper("emscripten_bind_Mat3_determinant_0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat3_inverse_0 = Module["_emscripten_bind_Mat3_inverse_0"] = createExportWrapper("emscripten_bind_Mat3_inverse_0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat3_invert_0 = Module["_emscripten_bind_Mat3_invert_0"] = createExportWrapper("emscripten_bind_Mat3_invert_0");

/** @type {function(...*):?} */
var _emscripten_bind_Mat3___destroy___0 = Module["_emscripten_bind_Mat3___destroy___0"] = createExportWrapper("emscripten_bind_Mat3___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_Collider_origin_0 = Module["_emscripten_bind_Collider_origin_0"] = createExportWrapper("emscripten_bind_Collider_origin_0");

/** @type {function(...*):?} */
var _emscripten_bind_Collider_getTransform_0 = Module["_emscripten_bind_Collider_getTransform_0"] = createExportWrapper("emscripten_bind_Collider_getTransform_0");

/** @type {function(...*):?} */
var _emscripten_bind_Collider_getInvTransform_0 = Module["_emscripten_bind_Collider_getInvTransform_0"] = createExportWrapper("emscripten_bind_Collider_getInvTransform_0");

/** @type {function(...*):?} */
var _emscripten_bind_Collider_setTransform_1 = Module["_emscripten_bind_Collider_setTransform_1"] = createExportWrapper("emscripten_bind_Collider_setTransform_1");

/** @type {function(...*):?} */
var _emscripten_bind_Collider_setInvTransform_1 = Module["_emscripten_bind_Collider_setInvTransform_1"] = createExportWrapper("emscripten_bind_Collider_setInvTransform_1");

/** @type {function(...*):?} */
var _emscripten_bind_Collider_collider2Object_1 = Module["_emscripten_bind_Collider_collider2Object_1"] = createExportWrapper("emscripten_bind_Collider_collider2Object_1");

/** @type {function(...*):?} */
var _emscripten_bind_Collider_object2Collider_1 = Module["_emscripten_bind_Collider_object2Collider_1"] = createExportWrapper("emscripten_bind_Collider_object2Collider_1");

/** @type {function(...*):?} */
var _emscripten_bind_Collider_rotCollider2Object_1 = Module["_emscripten_bind_Collider_rotCollider2Object_1"] = createExportWrapper("emscripten_bind_Collider_rotCollider2Object_1");

/** @type {function(...*):?} */
var _emscripten_bind_Collider_rotObject2Collider_1 = Module["_emscripten_bind_Collider_rotObject2Collider_1"] = createExportWrapper("emscripten_bind_Collider_rotObject2Collider_1");

/** @type {function(...*):?} */
var _emscripten_bind_Collider___destroy___0 = Module["_emscripten_bind_Collider___destroy___0"] = createExportWrapper("emscripten_bind_Collider___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_Rigidbody_0 = Module["_emscripten_bind_Rigidbody_Rigidbody_0"] = createExportWrapper("emscripten_bind_Rigidbody_Rigidbody_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getPos_0 = Module["_emscripten_bind_Rigidbody_getPos_0"] = createExportWrapper("emscripten_bind_Rigidbody_getPos_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getVel_0 = Module["_emscripten_bind_Rigidbody_getVel_0"] = createExportWrapper("emscripten_bind_Rigidbody_getVel_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getAcc_0 = Module["_emscripten_bind_Rigidbody_getAcc_0"] = createExportWrapper("emscripten_bind_Rigidbody_getAcc_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getAngle_0 = Module["_emscripten_bind_Rigidbody_getAngle_0"] = createExportWrapper("emscripten_bind_Rigidbody_getAngle_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getAngVel_0 = Module["_emscripten_bind_Rigidbody_getAngVel_0"] = createExportWrapper("emscripten_bind_Rigidbody_getAngVel_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getMass_0 = Module["_emscripten_bind_Rigidbody_getMass_0"] = createExportWrapper("emscripten_bind_Rigidbody_getMass_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getInvMass_0 = Module["_emscripten_bind_Rigidbody_getInvMass_0"] = createExportWrapper("emscripten_bind_Rigidbody_getInvMass_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getInertia_0 = Module["_emscripten_bind_Rigidbody_getInertia_0"] = createExportWrapper("emscripten_bind_Rigidbody_getInertia_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getInvInertia_0 = Module["_emscripten_bind_Rigidbody_getInvInertia_0"] = createExportWrapper("emscripten_bind_Rigidbody_getInvInertia_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getCollider_0 = Module["_emscripten_bind_Rigidbody_getCollider_0"] = createExportWrapper("emscripten_bind_Rigidbody_getCollider_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getRestitution_0 = Module["_emscripten_bind_Rigidbody_getRestitution_0"] = createExportWrapper("emscripten_bind_Rigidbody_getRestitution_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getTransform_0 = Module["_emscripten_bind_Rigidbody_getTransform_0"] = createExportWrapper("emscripten_bind_Rigidbody_getTransform_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_getInvTransform_0 = Module["_emscripten_bind_Rigidbody_getInvTransform_0"] = createExportWrapper("emscripten_bind_Rigidbody_getInvTransform_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_setPos_1 = Module["_emscripten_bind_Rigidbody_setPos_1"] = createExportWrapper("emscripten_bind_Rigidbody_setPos_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_setVel_1 = Module["_emscripten_bind_Rigidbody_setVel_1"] = createExportWrapper("emscripten_bind_Rigidbody_setVel_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_setAcc_1 = Module["_emscripten_bind_Rigidbody_setAcc_1"] = createExportWrapper("emscripten_bind_Rigidbody_setAcc_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_setAngle_1 = Module["_emscripten_bind_Rigidbody_setAngle_1"] = createExportWrapper("emscripten_bind_Rigidbody_setAngle_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_setAngVel_1 = Module["_emscripten_bind_Rigidbody_setAngVel_1"] = createExportWrapper("emscripten_bind_Rigidbody_setAngVel_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_setMass_1 = Module["_emscripten_bind_Rigidbody_setMass_1"] = createExportWrapper("emscripten_bind_Rigidbody_setMass_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_setInvMass_1 = Module["_emscripten_bind_Rigidbody_setInvMass_1"] = createExportWrapper("emscripten_bind_Rigidbody_setInvMass_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_setInertia_1 = Module["_emscripten_bind_Rigidbody_setInertia_1"] = createExportWrapper("emscripten_bind_Rigidbody_setInertia_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_setInvInertia_1 = Module["_emscripten_bind_Rigidbody_setInvInertia_1"] = createExportWrapper("emscripten_bind_Rigidbody_setInvInertia_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_setCollider_1 = Module["_emscripten_bind_Rigidbody_setCollider_1"] = createExportWrapper("emscripten_bind_Rigidbody_setCollider_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_setRestitution_1 = Module["_emscripten_bind_Rigidbody_setRestitution_1"] = createExportWrapper("emscripten_bind_Rigidbody_setRestitution_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_world2Local_1 = Module["_emscripten_bind_Rigidbody_world2Local_1"] = createExportWrapper("emscripten_bind_Rigidbody_world2Local_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_local2World_1 = Module["_emscripten_bind_Rigidbody_local2World_1"] = createExportWrapper("emscripten_bind_Rigidbody_local2World_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_addForce_1 = Module["_emscripten_bind_Rigidbody_addForce_1"] = createExportWrapper("emscripten_bind_Rigidbody_addForce_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_addForceAt_2 = Module["_emscripten_bind_Rigidbody_addForceAt_2"] = createExportWrapper("emscripten_bind_Rigidbody_addForceAt_2");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_addForceAtLocal_2 = Module["_emscripten_bind_Rigidbody_addForceAtLocal_2"] = createExportWrapper("emscripten_bind_Rigidbody_addForceAtLocal_2");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_addTorque_1 = Module["_emscripten_bind_Rigidbody_addTorque_1"] = createExportWrapper("emscripten_bind_Rigidbody_addTorque_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_clearAccums_0 = Module["_emscripten_bind_Rigidbody_clearAccums_0"] = createExportWrapper("emscripten_bind_Rigidbody_clearAccums_0");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody_step_1 = Module["_emscripten_bind_Rigidbody_step_1"] = createExportWrapper("emscripten_bind_Rigidbody_step_1");

/** @type {function(...*):?} */
var _emscripten_bind_Rigidbody___destroy___0 = Module["_emscripten_bind_Rigidbody___destroy___0"] = createExportWrapper("emscripten_bind_Rigidbody___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_ContactGenerator_generateContacts_0 = Module["_emscripten_bind_ContactGenerator_generateContacts_0"] = createExportWrapper("emscripten_bind_ContactGenerator_generateContacts_0");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_CircleCollider_2 = Module["_emscripten_bind_CircleCollider_CircleCollider_2"] = createExportWrapper("emscripten_bind_CircleCollider_CircleCollider_2");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_CircleCollider_3 = Module["_emscripten_bind_CircleCollider_CircleCollider_3"] = createExportWrapper("emscripten_bind_CircleCollider_CircleCollider_3");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_origin_0 = Module["_emscripten_bind_CircleCollider_origin_0"] = createExportWrapper("emscripten_bind_CircleCollider_origin_0");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_getTransform_0 = Module["_emscripten_bind_CircleCollider_getTransform_0"] = createExportWrapper("emscripten_bind_CircleCollider_getTransform_0");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_getInvTransform_0 = Module["_emscripten_bind_CircleCollider_getInvTransform_0"] = createExportWrapper("emscripten_bind_CircleCollider_getInvTransform_0");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_setTransform_1 = Module["_emscripten_bind_CircleCollider_setTransform_1"] = createExportWrapper("emscripten_bind_CircleCollider_setTransform_1");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_setInvTransform_1 = Module["_emscripten_bind_CircleCollider_setInvTransform_1"] = createExportWrapper("emscripten_bind_CircleCollider_setInvTransform_1");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_collider2Object_1 = Module["_emscripten_bind_CircleCollider_collider2Object_1"] = createExportWrapper("emscripten_bind_CircleCollider_collider2Object_1");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_object2Collider_1 = Module["_emscripten_bind_CircleCollider_object2Collider_1"] = createExportWrapper("emscripten_bind_CircleCollider_object2Collider_1");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_rotCollider2Object_1 = Module["_emscripten_bind_CircleCollider_rotCollider2Object_1"] = createExportWrapper("emscripten_bind_CircleCollider_rotCollider2Object_1");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_rotObject2Collider_1 = Module["_emscripten_bind_CircleCollider_rotObject2Collider_1"] = createExportWrapper("emscripten_bind_CircleCollider_rotObject2Collider_1");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_get_radius_0 = Module["_emscripten_bind_CircleCollider_get_radius_0"] = createExportWrapper("emscripten_bind_CircleCollider_get_radius_0");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider_set_radius_1 = Module["_emscripten_bind_CircleCollider_set_radius_1"] = createExportWrapper("emscripten_bind_CircleCollider_set_radius_1");

/** @type {function(...*):?} */
var _emscripten_bind_CircleCollider___destroy___0 = Module["_emscripten_bind_CircleCollider___destroy___0"] = createExportWrapper("emscripten_bind_CircleCollider___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_BoxCollider_2 = Module["_emscripten_bind_BoxCollider_BoxCollider_2"] = createExportWrapper("emscripten_bind_BoxCollider_BoxCollider_2");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_BoxCollider_3 = Module["_emscripten_bind_BoxCollider_BoxCollider_3"] = createExportWrapper("emscripten_bind_BoxCollider_BoxCollider_3");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_origin_0 = Module["_emscripten_bind_BoxCollider_origin_0"] = createExportWrapper("emscripten_bind_BoxCollider_origin_0");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_getTransform_0 = Module["_emscripten_bind_BoxCollider_getTransform_0"] = createExportWrapper("emscripten_bind_BoxCollider_getTransform_0");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_getInvTransform_0 = Module["_emscripten_bind_BoxCollider_getInvTransform_0"] = createExportWrapper("emscripten_bind_BoxCollider_getInvTransform_0");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_setTransform_1 = Module["_emscripten_bind_BoxCollider_setTransform_1"] = createExportWrapper("emscripten_bind_BoxCollider_setTransform_1");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_setInvTransform_1 = Module["_emscripten_bind_BoxCollider_setInvTransform_1"] = createExportWrapper("emscripten_bind_BoxCollider_setInvTransform_1");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_collider2Object_1 = Module["_emscripten_bind_BoxCollider_collider2Object_1"] = createExportWrapper("emscripten_bind_BoxCollider_collider2Object_1");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_object2Collider_1 = Module["_emscripten_bind_BoxCollider_object2Collider_1"] = createExportWrapper("emscripten_bind_BoxCollider_object2Collider_1");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_rotCollider2Object_1 = Module["_emscripten_bind_BoxCollider_rotCollider2Object_1"] = createExportWrapper("emscripten_bind_BoxCollider_rotCollider2Object_1");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_rotObject2Collider_1 = Module["_emscripten_bind_BoxCollider_rotObject2Collider_1"] = createExportWrapper("emscripten_bind_BoxCollider_rotObject2Collider_1");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_get_halfSize_0 = Module["_emscripten_bind_BoxCollider_get_halfSize_0"] = createExportWrapper("emscripten_bind_BoxCollider_get_halfSize_0");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider_set_halfSize_1 = Module["_emscripten_bind_BoxCollider_set_halfSize_1"] = createExportWrapper("emscripten_bind_BoxCollider_set_halfSize_1");

/** @type {function(...*):?} */
var _emscripten_bind_BoxCollider___destroy___0 = Module["_emscripten_bind_BoxCollider___destroy___0"] = createExportWrapper("emscripten_bind_BoxCollider___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_IntersectionDetector_circleCircle_2 = Module["_emscripten_bind_IntersectionDetector_circleCircle_2"] = createExportWrapper("emscripten_bind_IntersectionDetector_circleCircle_2");

/** @type {function(...*):?} */
var _emscripten_bind_IntersectionDetector_boxCircle_2 = Module["_emscripten_bind_IntersectionDetector_boxCircle_2"] = createExportWrapper("emscripten_bind_IntersectionDetector_boxCircle_2");

/** @type {function(...*):?} */
var _emscripten_bind_IntersectionDetector_boxBox_2 = Module["_emscripten_bind_IntersectionDetector_boxBox_2"] = createExportWrapper("emscripten_bind_IntersectionDetector_boxBox_2");

/** @type {function(...*):?} */
var _emscripten_bind_ForceGenerator_updateForce_2 = Module["_emscripten_bind_ForceGenerator_updateForce_2"] = createExportWrapper("emscripten_bind_ForceGenerator_updateForce_2");

/** @type {function(...*):?} */
var _emscripten_bind_Gravity_Gravity_1 = Module["_emscripten_bind_Gravity_Gravity_1"] = createExportWrapper("emscripten_bind_Gravity_Gravity_1");

/** @type {function(...*):?} */
var _emscripten_bind_Gravity_updateForce_2 = Module["_emscripten_bind_Gravity_updateForce_2"] = createExportWrapper("emscripten_bind_Gravity_updateForce_2");

/** @type {function(...*):?} */
var _emscripten_bind_Gravity___destroy___0 = Module["_emscripten_bind_Gravity___destroy___0"] = createExportWrapper("emscripten_bind_Gravity___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_Spring_Spring_5 = Module["_emscripten_bind_Spring_Spring_5"] = createExportWrapper("emscripten_bind_Spring_Spring_5");

/** @type {function(...*):?} */
var _emscripten_bind_Spring_getEnd_0 = Module["_emscripten_bind_Spring_getEnd_0"] = createExportWrapper("emscripten_bind_Spring_getEnd_0");

/** @type {function(...*):?} */
var _emscripten_bind_Spring_setEnd_1 = Module["_emscripten_bind_Spring_setEnd_1"] = createExportWrapper("emscripten_bind_Spring_setEnd_1");

/** @type {function(...*):?} */
var _emscripten_bind_Spring_updateForce_2 = Module["_emscripten_bind_Spring_updateForce_2"] = createExportWrapper("emscripten_bind_Spring_updateForce_2");

/** @type {function(...*):?} */
var _emscripten_bind_Spring_get_k_0 = Module["_emscripten_bind_Spring_get_k_0"] = createExportWrapper("emscripten_bind_Spring_get_k_0");

/** @type {function(...*):?} */
var _emscripten_bind_Spring_set_k_1 = Module["_emscripten_bind_Spring_set_k_1"] = createExportWrapper("emscripten_bind_Spring_set_k_1");

/** @type {function(...*):?} */
var _emscripten_bind_Spring_get_length_0 = Module["_emscripten_bind_Spring_get_length_0"] = createExportWrapper("emscripten_bind_Spring_get_length_0");

/** @type {function(...*):?} */
var _emscripten_bind_Spring_set_length_1 = Module["_emscripten_bind_Spring_set_length_1"] = createExportWrapper("emscripten_bind_Spring_set_length_1");

/** @type {function(...*):?} */
var _emscripten_bind_Spring___destroy___0 = Module["_emscripten_bind_Spring___destroy___0"] = createExportWrapper("emscripten_bind_Spring___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_World_World_0 = Module["_emscripten_bind_World_World_0"] = createExportWrapper("emscripten_bind_World_World_0");

/** @type {function(...*):?} */
var _emscripten_bind_World_addBody_2 = Module["_emscripten_bind_World_addBody_2"] = createExportWrapper("emscripten_bind_World_addBody_2");

/** @type {function(...*):?} */
var _emscripten_bind_World_removeBody_2 = Module["_emscripten_bind_World_removeBody_2"] = createExportWrapper("emscripten_bind_World_removeBody_2");

/** @type {function(...*):?} */
var _emscripten_bind_World_addFGen_2 = Module["_emscripten_bind_World_addFGen_2"] = createExportWrapper("emscripten_bind_World_addFGen_2");

/** @type {function(...*):?} */
var _emscripten_bind_World_removeFGen_2 = Module["_emscripten_bind_World_removeFGen_2"] = createExportWrapper("emscripten_bind_World_removeFGen_2");

/** @type {function(...*):?} */
var _emscripten_bind_World_addContactGen_1 = Module["_emscripten_bind_World_addContactGen_1"] = createExportWrapper("emscripten_bind_World_addContactGen_1");

/** @type {function(...*):?} */
var _emscripten_bind_World_removeContactGen_1 = Module["_emscripten_bind_World_removeContactGen_1"] = createExportWrapper("emscripten_bind_World_removeContactGen_1");

/** @type {function(...*):?} */
var _emscripten_bind_World_step_1 = Module["_emscripten_bind_World_step_1"] = createExportWrapper("emscripten_bind_World_step_1");

/** @type {function(...*):?} */
var _emscripten_bind_World___destroy___0 = Module["_emscripten_bind_World___destroy___0"] = createExportWrapper("emscripten_bind_World___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_Particle_0 = Module["_emscripten_bind_Particle_Particle_0"] = createExportWrapper("emscripten_bind_Particle_Particle_0");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_Particle_1 = Module["_emscripten_bind_Particle_Particle_1"] = createExportWrapper("emscripten_bind_Particle_Particle_1");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_Particle_2 = Module["_emscripten_bind_Particle_Particle_2"] = createExportWrapper("emscripten_bind_Particle_Particle_2");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_getPos_0 = Module["_emscripten_bind_Particle_getPos_0"] = createExportWrapper("emscripten_bind_Particle_getPos_0");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_getVel_0 = Module["_emscripten_bind_Particle_getVel_0"] = createExportWrapper("emscripten_bind_Particle_getVel_0");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_getAcc_0 = Module["_emscripten_bind_Particle_getAcc_0"] = createExportWrapper("emscripten_bind_Particle_getAcc_0");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_getMass_0 = Module["_emscripten_bind_Particle_getMass_0"] = createExportWrapper("emscripten_bind_Particle_getMass_0");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_getInvMass_0 = Module["_emscripten_bind_Particle_getInvMass_0"] = createExportWrapper("emscripten_bind_Particle_getInvMass_0");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_getDamping_0 = Module["_emscripten_bind_Particle_getDamping_0"] = createExportWrapper("emscripten_bind_Particle_getDamping_0");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_setPos_1 = Module["_emscripten_bind_Particle_setPos_1"] = createExportWrapper("emscripten_bind_Particle_setPos_1");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_setVel_1 = Module["_emscripten_bind_Particle_setVel_1"] = createExportWrapper("emscripten_bind_Particle_setVel_1");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_setAcc_1 = Module["_emscripten_bind_Particle_setAcc_1"] = createExportWrapper("emscripten_bind_Particle_setAcc_1");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_setMass_1 = Module["_emscripten_bind_Particle_setMass_1"] = createExportWrapper("emscripten_bind_Particle_setMass_1");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_setInvMass_1 = Module["_emscripten_bind_Particle_setInvMass_1"] = createExportWrapper("emscripten_bind_Particle_setInvMass_1");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_setDamping_1 = Module["_emscripten_bind_Particle_setDamping_1"] = createExportWrapper("emscripten_bind_Particle_setDamping_1");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_addForce_1 = Module["_emscripten_bind_Particle_addForce_1"] = createExportWrapper("emscripten_bind_Particle_addForce_1");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_addImpulse_1 = Module["_emscripten_bind_Particle_addImpulse_1"] = createExportWrapper("emscripten_bind_Particle_addImpulse_1");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_clearForceAccum_0 = Module["_emscripten_bind_Particle_clearForceAccum_0"] = createExportWrapper("emscripten_bind_Particle_clearForceAccum_0");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_setStatic_0 = Module["_emscripten_bind_Particle_setStatic_0"] = createExportWrapper("emscripten_bind_Particle_setStatic_0");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_isStatic_0 = Module["_emscripten_bind_Particle_isStatic_0"] = createExportWrapper("emscripten_bind_Particle_isStatic_0");

/** @type {function(...*):?} */
var _emscripten_bind_Particle_step_1 = Module["_emscripten_bind_Particle_step_1"] = createExportWrapper("emscripten_bind_Particle_step_1");

/** @type {function(...*):?} */
var _emscripten_bind_Particle___destroy___0 = Module["_emscripten_bind_Particle___destroy___0"] = createExportWrapper("emscripten_bind_Particle___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleForceGenerator_updateForce_2 = Module["_emscripten_bind_ParticleForceGenerator_updateForce_2"] = createExportWrapper("emscripten_bind_ParticleForceGenerator_updateForce_2");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleGravity_ParticleGravity_1 = Module["_emscripten_bind_ParticleGravity_ParticleGravity_1"] = createExportWrapper("emscripten_bind_ParticleGravity_ParticleGravity_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleGravity_updateForce_2 = Module["_emscripten_bind_ParticleGravity_updateForce_2"] = createExportWrapper("emscripten_bind_ParticleGravity_updateForce_2");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleGravity___destroy___0 = Module["_emscripten_bind_ParticleGravity___destroy___0"] = createExportWrapper("emscripten_bind_ParticleGravity___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleDrag_ParticleDrag_2 = Module["_emscripten_bind_ParticleDrag_ParticleDrag_2"] = createExportWrapper("emscripten_bind_ParticleDrag_ParticleDrag_2");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleDrag_updateForce_2 = Module["_emscripten_bind_ParticleDrag_updateForce_2"] = createExportWrapper("emscripten_bind_ParticleDrag_updateForce_2");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleDrag___destroy___0 = Module["_emscripten_bind_ParticleDrag___destroy___0"] = createExportWrapper("emscripten_bind_ParticleDrag___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleSpring_ParticleSpring_3 = Module["_emscripten_bind_ParticleSpring_ParticleSpring_3"] = createExportWrapper("emscripten_bind_ParticleSpring_ParticleSpring_3");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleSpring_setEnd_1 = Module["_emscripten_bind_ParticleSpring_setEnd_1"] = createExportWrapper("emscripten_bind_ParticleSpring_setEnd_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleSpring_getEnd_0 = Module["_emscripten_bind_ParticleSpring_getEnd_0"] = createExportWrapper("emscripten_bind_ParticleSpring_getEnd_0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleSpring_updateForce_2 = Module["_emscripten_bind_ParticleSpring_updateForce_2"] = createExportWrapper("emscripten_bind_ParticleSpring_updateForce_2");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleSpring_get_k_0 = Module["_emscripten_bind_ParticleSpring_get_k_0"] = createExportWrapper("emscripten_bind_ParticleSpring_get_k_0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleSpring_set_k_1 = Module["_emscripten_bind_ParticleSpring_set_k_1"] = createExportWrapper("emscripten_bind_ParticleSpring_set_k_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleSpring_get_length_0 = Module["_emscripten_bind_ParticleSpring_get_length_0"] = createExportWrapper("emscripten_bind_ParticleSpring_get_length_0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleSpring_set_length_1 = Module["_emscripten_bind_ParticleSpring_set_length_1"] = createExportWrapper("emscripten_bind_ParticleSpring_set_length_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleSpring___destroy___0 = Module["_emscripten_bind_ParticleSpring___destroy___0"] = createExportWrapper("emscripten_bind_ParticleSpring___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleContactGenerator_generateContacts_0 = Module["_emscripten_bind_ParticleContactGenerator_generateContacts_0"] = createExportWrapper("emscripten_bind_ParticleContactGenerator_generateContacts_0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleLink_setParticle_2 = Module["_emscripten_bind_ParticleLink_setParticle_2"] = createExportWrapper("emscripten_bind_ParticleLink_setParticle_2");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleLink_getParticle_1 = Module["_emscripten_bind_ParticleLink_getParticle_1"] = createExportWrapper("emscripten_bind_ParticleLink_getParticle_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleLink_generateContacts_0 = Module["_emscripten_bind_ParticleLink_generateContacts_0"] = createExportWrapper("emscripten_bind_ParticleLink_generateContacts_0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleCable_ParticleCable_2 = Module["_emscripten_bind_ParticleCable_ParticleCable_2"] = createExportWrapper("emscripten_bind_ParticleCable_ParticleCable_2");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleCable_setParticle_2 = Module["_emscripten_bind_ParticleCable_setParticle_2"] = createExportWrapper("emscripten_bind_ParticleCable_setParticle_2");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleCable_getParticle_1 = Module["_emscripten_bind_ParticleCable_getParticle_1"] = createExportWrapper("emscripten_bind_ParticleCable_getParticle_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleCable_generateContacts_0 = Module["_emscripten_bind_ParticleCable_generateContacts_0"] = createExportWrapper("emscripten_bind_ParticleCable_generateContacts_0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleCable_get_length_0 = Module["_emscripten_bind_ParticleCable_get_length_0"] = createExportWrapper("emscripten_bind_ParticleCable_get_length_0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleCable_set_length_1 = Module["_emscripten_bind_ParticleCable_set_length_1"] = createExportWrapper("emscripten_bind_ParticleCable_set_length_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleCable_get_restitution_0 = Module["_emscripten_bind_ParticleCable_get_restitution_0"] = createExportWrapper("emscripten_bind_ParticleCable_get_restitution_0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleCable_set_restitution_1 = Module["_emscripten_bind_ParticleCable_set_restitution_1"] = createExportWrapper("emscripten_bind_ParticleCable_set_restitution_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleCable___destroy___0 = Module["_emscripten_bind_ParticleCable___destroy___0"] = createExportWrapper("emscripten_bind_ParticleCable___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleRod_ParticleRod_1 = Module["_emscripten_bind_ParticleRod_ParticleRod_1"] = createExportWrapper("emscripten_bind_ParticleRod_ParticleRod_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleRod_setParticle_2 = Module["_emscripten_bind_ParticleRod_setParticle_2"] = createExportWrapper("emscripten_bind_ParticleRod_setParticle_2");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleRod_getParticle_1 = Module["_emscripten_bind_ParticleRod_getParticle_1"] = createExportWrapper("emscripten_bind_ParticleRod_getParticle_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleRod_generateContacts_0 = Module["_emscripten_bind_ParticleRod_generateContacts_0"] = createExportWrapper("emscripten_bind_ParticleRod_generateContacts_0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleRod_get_length_0 = Module["_emscripten_bind_ParticleRod_get_length_0"] = createExportWrapper("emscripten_bind_ParticleRod_get_length_0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleRod_set_length_1 = Module["_emscripten_bind_ParticleRod_set_length_1"] = createExportWrapper("emscripten_bind_ParticleRod_set_length_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleRod___destroy___0 = Module["_emscripten_bind_ParticleRod___destroy___0"] = createExportWrapper("emscripten_bind_ParticleRod___destroy___0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleWorld_ParticleWorld_0 = Module["_emscripten_bind_ParticleWorld_ParticleWorld_0"] = createExportWrapper("emscripten_bind_ParticleWorld_ParticleWorld_0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleWorld_ParticleWorld_1 = Module["_emscripten_bind_ParticleWorld_ParticleWorld_1"] = createExportWrapper("emscripten_bind_ParticleWorld_ParticleWorld_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleWorld_addParticle_1 = Module["_emscripten_bind_ParticleWorld_addParticle_1"] = createExportWrapper("emscripten_bind_ParticleWorld_addParticle_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleWorld_removeParticle_1 = Module["_emscripten_bind_ParticleWorld_removeParticle_1"] = createExportWrapper("emscripten_bind_ParticleWorld_removeParticle_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleWorld_addPFGen_2 = Module["_emscripten_bind_ParticleWorld_addPFGen_2"] = createExportWrapper("emscripten_bind_ParticleWorld_addPFGen_2");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleWorld_removePFGen_2 = Module["_emscripten_bind_ParticleWorld_removePFGen_2"] = createExportWrapper("emscripten_bind_ParticleWorld_removePFGen_2");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleWorld_addPContactGenerator_1 = Module["_emscripten_bind_ParticleWorld_addPContactGenerator_1"] = createExportWrapper("emscripten_bind_ParticleWorld_addPContactGenerator_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleWorld_removePContactGenerator_1 = Module["_emscripten_bind_ParticleWorld_removePContactGenerator_1"] = createExportWrapper("emscripten_bind_ParticleWorld_removePContactGenerator_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleWorld_generateContacts_0 = Module["_emscripten_bind_ParticleWorld_generateContacts_0"] = createExportWrapper("emscripten_bind_ParticleWorld_generateContacts_0");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleWorld_step_1 = Module["_emscripten_bind_ParticleWorld_step_1"] = createExportWrapper("emscripten_bind_ParticleWorld_step_1");

/** @type {function(...*):?} */
var _emscripten_bind_ParticleWorld___destroy___0 = Module["_emscripten_bind_ParticleWorld___destroy___0"] = createExportWrapper("emscripten_bind_ParticleWorld___destroy___0");

/** @type {function(...*):?} */
var ___errno_location = Module["___errno_location"] = createExportWrapper("__errno_location");

/** @type {function(...*):?} */
var _fflush = Module["_fflush"] = createExportWrapper("fflush");

/** @type {function(...*):?} */
var _malloc = Module["_malloc"] = createExportWrapper("malloc");

/** @type {function(...*):?} */
var _emscripten_stack_init = Module["_emscripten_stack_init"] = function() {
  return (_emscripten_stack_init = Module["_emscripten_stack_init"] = Module["asm"]["emscripten_stack_init"]).apply(null, arguments);
};

/** @type {function(...*):?} */
var _emscripten_stack_get_free = Module["_emscripten_stack_get_free"] = function() {
  return (_emscripten_stack_get_free = Module["_emscripten_stack_get_free"] = Module["asm"]["emscripten_stack_get_free"]).apply(null, arguments);
};

/** @type {function(...*):?} */
var _emscripten_stack_get_base = Module["_emscripten_stack_get_base"] = function() {
  return (_emscripten_stack_get_base = Module["_emscripten_stack_get_base"] = Module["asm"]["emscripten_stack_get_base"]).apply(null, arguments);
};

/** @type {function(...*):?} */
var _emscripten_stack_get_end = Module["_emscripten_stack_get_end"] = function() {
  return (_emscripten_stack_get_end = Module["_emscripten_stack_get_end"] = Module["asm"]["emscripten_stack_get_end"]).apply(null, arguments);
};

/** @type {function(...*):?} */
var stackSave = Module["stackSave"] = createExportWrapper("stackSave");

/** @type {function(...*):?} */
var stackRestore = Module["stackRestore"] = createExportWrapper("stackRestore");

/** @type {function(...*):?} */
var stackAlloc = Module["stackAlloc"] = createExportWrapper("stackAlloc");

/** @type {function(...*):?} */
var ___cxa_is_pointer_type = Module["___cxa_is_pointer_type"] = createExportWrapper("__cxa_is_pointer_type");





// === Auto-generated postamble setup entry stuff ===

unexportedRuntimeFunction('ccall', false);
unexportedRuntimeFunction('cwrap', false);
unexportedRuntimeFunction('allocate', false);
unexportedRuntimeFunction('UTF8ArrayToString', false);
unexportedRuntimeFunction('UTF8ToString', false);
unexportedRuntimeFunction('stringToUTF8Array', false);
unexportedRuntimeFunction('stringToUTF8', false);
unexportedRuntimeFunction('lengthBytesUTF8', false);
unexportedRuntimeFunction('addOnPreRun', false);
unexportedRuntimeFunction('addOnInit', false);
unexportedRuntimeFunction('addOnPreMain', false);
unexportedRuntimeFunction('addOnExit', false);
unexportedRuntimeFunction('addOnPostRun', false);
unexportedRuntimeFunction('addRunDependency', true);
unexportedRuntimeFunction('removeRunDependency', true);
unexportedRuntimeFunction('FS_createFolder', false);
unexportedRuntimeFunction('FS_createPath', true);
unexportedRuntimeFunction('FS_createDataFile', true);
unexportedRuntimeFunction('FS_createPreloadedFile', true);
unexportedRuntimeFunction('FS_createLazyFile', true);
unexportedRuntimeFunction('FS_createLink', false);
unexportedRuntimeFunction('FS_createDevice', true);
unexportedRuntimeFunction('FS_unlink', true);
unexportedRuntimeFunction('getLEB', false);
unexportedRuntimeFunction('getFunctionTables', false);
unexportedRuntimeFunction('alignFunctionTables', false);
unexportedRuntimeFunction('registerFunctions', false);
unexportedRuntimeFunction('addFunction', false);
unexportedRuntimeFunction('removeFunction', false);
unexportedRuntimeFunction('prettyPrint', false);
unexportedRuntimeFunction('getCompilerSetting', false);
unexportedRuntimeFunction('print', false);
unexportedRuntimeFunction('printErr', false);
unexportedRuntimeFunction('getTempRet0', false);
unexportedRuntimeFunction('setTempRet0', false);
unexportedRuntimeFunction('callMain', false);
unexportedRuntimeFunction('abort', false);
unexportedRuntimeFunction('keepRuntimeAlive', false);
unexportedRuntimeFunction('wasmMemory', false);
unexportedRuntimeFunction('warnOnce', false);
unexportedRuntimeFunction('stackSave', false);
unexportedRuntimeFunction('stackRestore', false);
unexportedRuntimeFunction('stackAlloc', false);
unexportedRuntimeFunction('AsciiToString', false);
unexportedRuntimeFunction('stringToAscii', false);
unexportedRuntimeFunction('UTF16ToString', false);
unexportedRuntimeFunction('stringToUTF16', false);
unexportedRuntimeFunction('lengthBytesUTF16', false);
unexportedRuntimeFunction('UTF32ToString', false);
unexportedRuntimeFunction('stringToUTF32', false);
unexportedRuntimeFunction('lengthBytesUTF32', false);
unexportedRuntimeFunction('allocateUTF8', false);
unexportedRuntimeFunction('allocateUTF8OnStack', false);
unexportedRuntimeFunction('ExitStatus', false);
unexportedRuntimeFunction('intArrayFromString', false);
unexportedRuntimeFunction('intArrayToString', false);
unexportedRuntimeFunction('writeStringToMemory', false);
unexportedRuntimeFunction('writeArrayToMemory', false);
unexportedRuntimeFunction('writeAsciiToMemory', false);
Module["writeStackCookie"] = writeStackCookie;
Module["checkStackCookie"] = checkStackCookie;
unexportedRuntimeFunction('ptrToString', false);
unexportedRuntimeFunction('zeroMemory', false);
unexportedRuntimeFunction('stringToNewUTF8', false);
unexportedRuntimeFunction('getHeapMax', false);
unexportedRuntimeFunction('abortOnCannotGrowMemory', false);
unexportedRuntimeFunction('emscripten_realloc_buffer', false);
unexportedRuntimeFunction('ENV', false);
unexportedRuntimeFunction('ERRNO_CODES', false);
unexportedRuntimeFunction('ERRNO_MESSAGES', false);
unexportedRuntimeFunction('setErrNo', false);
unexportedRuntimeFunction('inetPton4', false);
unexportedRuntimeFunction('inetNtop4', false);
unexportedRuntimeFunction('inetPton6', false);
unexportedRuntimeFunction('inetNtop6', false);
unexportedRuntimeFunction('readSockaddr', false);
unexportedRuntimeFunction('writeSockaddr', false);
unexportedRuntimeFunction('DNS', false);
unexportedRuntimeFunction('getHostByName', false);
unexportedRuntimeFunction('Protocols', false);
unexportedRuntimeFunction('Sockets', false);
unexportedRuntimeFunction('getRandomDevice', false);
unexportedRuntimeFunction('traverseStack', false);
unexportedRuntimeFunction('UNWIND_CACHE', false);
unexportedRuntimeFunction('convertPCtoSourceLocation', false);
unexportedRuntimeFunction('readAsmConstArgsArray', false);
unexportedRuntimeFunction('readAsmConstArgs', false);
unexportedRuntimeFunction('mainThreadEM_ASM', false);
unexportedRuntimeFunction('jstoi_q', false);
unexportedRuntimeFunction('jstoi_s', false);
unexportedRuntimeFunction('getExecutableName', false);
unexportedRuntimeFunction('listenOnce', false);
unexportedRuntimeFunction('autoResumeAudioContext', false);
unexportedRuntimeFunction('dynCallLegacy', false);
unexportedRuntimeFunction('getDynCaller', false);
unexportedRuntimeFunction('dynCall', false);
unexportedRuntimeFunction('handleException', false);
unexportedRuntimeFunction('runtimeKeepalivePush', false);
unexportedRuntimeFunction('runtimeKeepalivePop', false);
unexportedRuntimeFunction('callUserCallback', false);
unexportedRuntimeFunction('maybeExit', false);
unexportedRuntimeFunction('safeSetTimeout', false);
unexportedRuntimeFunction('asmjsMangle', false);
unexportedRuntimeFunction('asyncLoad', false);
unexportedRuntimeFunction('alignMemory', false);
unexportedRuntimeFunction('mmapAlloc', false);
unexportedRuntimeFunction('writeI53ToI64', false);
unexportedRuntimeFunction('writeI53ToI64Clamped', false);
unexportedRuntimeFunction('writeI53ToI64Signaling', false);
unexportedRuntimeFunction('writeI53ToU64Clamped', false);
unexportedRuntimeFunction('writeI53ToU64Signaling', false);
unexportedRuntimeFunction('readI53FromI64', false);
unexportedRuntimeFunction('readI53FromU64', false);
unexportedRuntimeFunction('convertI32PairToI53', false);
unexportedRuntimeFunction('convertI32PairToI53Checked', false);
unexportedRuntimeFunction('convertU32PairToI53', false);
unexportedRuntimeFunction('reallyNegative', false);
unexportedRuntimeFunction('unSign', false);
unexportedRuntimeFunction('strLen', false);
unexportedRuntimeFunction('reSign', false);
unexportedRuntimeFunction('formatString', false);
unexportedRuntimeFunction('setValue', false);
unexportedRuntimeFunction('getValue', false);
unexportedRuntimeFunction('PATH', false);
unexportedRuntimeFunction('PATH_FS', false);
unexportedRuntimeFunction('SYSCALLS', false);
unexportedRuntimeFunction('getSocketFromFD', false);
unexportedRuntimeFunction('getSocketAddress', false);
unexportedRuntimeFunction('JSEvents', false);
unexportedRuntimeFunction('registerKeyEventCallback', false);
unexportedRuntimeFunction('specialHTMLTargets', false);
unexportedRuntimeFunction('maybeCStringToJsString', false);
unexportedRuntimeFunction('findEventTarget', false);
unexportedRuntimeFunction('findCanvasEventTarget', false);
unexportedRuntimeFunction('getBoundingClientRect', false);
unexportedRuntimeFunction('fillMouseEventData', false);
unexportedRuntimeFunction('registerMouseEventCallback', false);
unexportedRuntimeFunction('registerWheelEventCallback', false);
unexportedRuntimeFunction('registerUiEventCallback', false);
unexportedRuntimeFunction('registerFocusEventCallback', false);
unexportedRuntimeFunction('fillDeviceOrientationEventData', false);
unexportedRuntimeFunction('registerDeviceOrientationEventCallback', false);
unexportedRuntimeFunction('fillDeviceMotionEventData', false);
unexportedRuntimeFunction('registerDeviceMotionEventCallback', false);
unexportedRuntimeFunction('screenOrientation', false);
unexportedRuntimeFunction('fillOrientationChangeEventData', false);
unexportedRuntimeFunction('registerOrientationChangeEventCallback', false);
unexportedRuntimeFunction('fillFullscreenChangeEventData', false);
unexportedRuntimeFunction('registerFullscreenChangeEventCallback', false);
unexportedRuntimeFunction('JSEvents_requestFullscreen', false);
unexportedRuntimeFunction('JSEvents_resizeCanvasForFullscreen', false);
unexportedRuntimeFunction('registerRestoreOldStyle', false);
unexportedRuntimeFunction('hideEverythingExceptGivenElement', false);
unexportedRuntimeFunction('restoreHiddenElements', false);
unexportedRuntimeFunction('setLetterbox', false);
unexportedRuntimeFunction('currentFullscreenStrategy', false);
unexportedRuntimeFunction('restoreOldWindowedStyle', false);
unexportedRuntimeFunction('softFullscreenResizeWebGLRenderTarget', false);
unexportedRuntimeFunction('doRequestFullscreen', false);
unexportedRuntimeFunction('fillPointerlockChangeEventData', false);
unexportedRuntimeFunction('registerPointerlockChangeEventCallback', false);
unexportedRuntimeFunction('registerPointerlockErrorEventCallback', false);
unexportedRuntimeFunction('requestPointerLock', false);
unexportedRuntimeFunction('fillVisibilityChangeEventData', false);
unexportedRuntimeFunction('registerVisibilityChangeEventCallback', false);
unexportedRuntimeFunction('registerTouchEventCallback', false);
unexportedRuntimeFunction('fillGamepadEventData', false);
unexportedRuntimeFunction('registerGamepadEventCallback', false);
unexportedRuntimeFunction('registerBeforeUnloadEventCallback', false);
unexportedRuntimeFunction('fillBatteryEventData', false);
unexportedRuntimeFunction('battery', false);
unexportedRuntimeFunction('registerBatteryEventCallback', false);
unexportedRuntimeFunction('setCanvasElementSize', false);
unexportedRuntimeFunction('getCanvasElementSize', false);
unexportedRuntimeFunction('demangle', false);
unexportedRuntimeFunction('demangleAll', false);
unexportedRuntimeFunction('jsStackTrace', false);
unexportedRuntimeFunction('stackTrace', false);
unexportedRuntimeFunction('getEnvStrings', false);
unexportedRuntimeFunction('checkWasiClock', false);
unexportedRuntimeFunction('flush_NO_FILESYSTEM', false);
unexportedRuntimeFunction('dlopenMissingError', false);
unexportedRuntimeFunction('setImmediateWrapped', false);
unexportedRuntimeFunction('clearImmediateWrapped', false);
unexportedRuntimeFunction('polyfillSetImmediate', false);
unexportedRuntimeFunction('uncaughtExceptionCount', false);
unexportedRuntimeFunction('exceptionLast', false);
unexportedRuntimeFunction('exceptionCaught', false);
unexportedRuntimeFunction('ExceptionInfo', false);
unexportedRuntimeFunction('exception_addRef', false);
unexportedRuntimeFunction('exception_decRef', false);
unexportedRuntimeFunction('Browser', false);
unexportedRuntimeFunction('setMainLoop', false);
unexportedRuntimeFunction('wget', false);
unexportedRuntimeFunction('FS', false);
unexportedRuntimeFunction('MEMFS', false);
unexportedRuntimeFunction('TTY', false);
unexportedRuntimeFunction('PIPEFS', false);
unexportedRuntimeFunction('SOCKFS', false);
unexportedRuntimeFunction('_setNetworkCallback', false);
unexportedRuntimeFunction('tempFixedLengthArray', false);
unexportedRuntimeFunction('miniTempWebGLFloatBuffers', false);
unexportedRuntimeFunction('heapObjectForWebGLType', false);
unexportedRuntimeFunction('heapAccessShiftForWebGLHeap', false);
unexportedRuntimeFunction('GL', false);
unexportedRuntimeFunction('emscriptenWebGLGet', false);
unexportedRuntimeFunction('computeUnpackAlignedImageSize', false);
unexportedRuntimeFunction('emscriptenWebGLGetTexPixelData', false);
unexportedRuntimeFunction('emscriptenWebGLGetUniform', false);
unexportedRuntimeFunction('webglGetUniformLocation', false);
unexportedRuntimeFunction('webglPrepareUniformLocationsBeforeFirstUse', false);
unexportedRuntimeFunction('webglGetLeftBracePos', false);
unexportedRuntimeFunction('emscriptenWebGLGetVertexAttrib', false);
unexportedRuntimeFunction('writeGLArray', false);
unexportedRuntimeFunction('AL', false);
unexportedRuntimeFunction('SDL_unicode', false);
unexportedRuntimeFunction('SDL_ttfContext', false);
unexportedRuntimeFunction('SDL_audio', false);
unexportedRuntimeFunction('SDL', false);
unexportedRuntimeFunction('SDL_gfx', false);
unexportedRuntimeFunction('GLUT', false);
unexportedRuntimeFunction('EGL', false);
unexportedRuntimeFunction('GLFW_Window', false);
unexportedRuntimeFunction('GLFW', false);
unexportedRuntimeFunction('GLEW', false);
unexportedRuntimeFunction('IDBStore', false);
unexportedRuntimeFunction('runAndAbortIfError', false);
unexportedRuntimeSymbol('ALLOC_NORMAL', false);
unexportedRuntimeSymbol('ALLOC_STACK', false);

var calledRun;

/**
 * @constructor
 * @this {ExitStatus}
 */
function ExitStatus(status) {
  this.name = "ExitStatus";
  this.message = "Program terminated with exit(" + status + ")";
  this.status = status;
}

var calledMain = false;

dependenciesFulfilled = function runCaller() {
  // If run has never been called, and we should call run (INVOKE_RUN is true, and Module.noInitialRun is not false)
  if (!calledRun) run();
  if (!calledRun) dependenciesFulfilled = runCaller; // try this again later, after new deps are fulfilled
};

function stackCheckInit() {
  // This is normally called automatically during __wasm_call_ctors but need to
  // get these values before even running any of the ctors so we call it redundantly
  // here.
  // TODO(sbc): Move writeStackCookie to native to to avoid this.
  _emscripten_stack_init();
  writeStackCookie();
}

/** @type {function(Array=)} */
function run(args) {
  args = args || arguments_;

  if (runDependencies > 0) {
    return;
  }

  stackCheckInit();

  preRun();

  // a preRun added a dependency, run will be called later
  if (runDependencies > 0) {
    return;
  }

  function doRun() {
    // run may have just been called through dependencies being fulfilled just in this very frame,
    // or while the async setStatus time below was happening
    if (calledRun) return;
    calledRun = true;
    Module['calledRun'] = true;

    if (ABORT) return;

    initRuntime();

    readyPromiseResolve(Module);
    if (Module['onRuntimeInitialized']) Module['onRuntimeInitialized']();

    assert(!Module['_main'], 'compiled without a main, but one is present. if you added it from JS, use Module["onRuntimeInitialized"]');

    postRun();
  }

  if (Module['setStatus']) {
    Module['setStatus']('Running...');
    setTimeout(function() {
      setTimeout(function() {
        Module['setStatus']('');
      }, 1);
      doRun();
    }, 1);
  } else
  {
    doRun();
  }
  checkStackCookie();
}
Module['run'] = run;

function checkUnflushedContent() {
  // Compiler settings do not allow exiting the runtime, so flushing
  // the streams is not possible. but in ASSERTIONS mode we check
  // if there was something to flush, and if so tell the user they
  // should request that the runtime be exitable.
  // Normally we would not even include flush() at all, but in ASSERTIONS
  // builds we do so just for this check, and here we see if there is any
  // content to flush, that is, we check if there would have been
  // something a non-ASSERTIONS build would have not seen.
  // How we flush the streams depends on whether we are in SYSCALLS_REQUIRE_FILESYSTEM=0
  // mode (which has its own special function for this; otherwise, all
  // the code is inside libc)
  var oldOut = out;
  var oldErr = err;
  var has = false;
  out = err = (x) => {
    has = true;
  }
  try { // it doesn't matter if it fails
    _fflush(0);
  } catch(e) {}
  out = oldOut;
  err = oldErr;
  if (has) {
    warnOnce('stdio streams had content in them that was not flushed. you should set EXIT_RUNTIME to 1 (see the FAQ), or make sure to emit a newline when you printf etc.');
    warnOnce('(this may also be due to not including full filesystem support - try building with -sFORCE_FILESYSTEM)');
  }
}

/** @param {boolean|number=} implicit */
function exit(status, implicit) {
  EXITSTATUS = status;

  checkUnflushedContent();

  // if exit() was called explicitly, warn the user if the runtime isn't actually being shut down
  if (keepRuntimeAlive() && !implicit) {
    var msg = 'program exited (with status: ' + status + '), but EXIT_RUNTIME is not set, so halting execution but not exiting the runtime or preventing further async execution (build with EXIT_RUNTIME=1, if you want a true shutdown)';
    readyPromiseReject(msg);
    err(msg);
  }

  procExit(status);
}

function procExit(code) {
  EXITSTATUS = code;
  if (!keepRuntimeAlive()) {
    if (Module['onExit']) Module['onExit'](code);
    ABORT = true;
  }
  quit_(code, new ExitStatus(code));
}

if (Module['preInit']) {
  if (typeof Module['preInit'] == 'function') Module['preInit'] = [Module['preInit']];
  while (Module['preInit'].length > 0) {
    Module['preInit'].pop()();
  }
}

run();






// Bindings utilities

/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
function WrapperObject() {
}
WrapperObject.prototype = Object.create(WrapperObject.prototype);
WrapperObject.prototype.constructor = WrapperObject;
WrapperObject.prototype.__class__ = WrapperObject;
WrapperObject.__cache__ = {};
Module['WrapperObject'] = WrapperObject;

/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant)
    @param {*=} __class__ */
function getCache(__class__) {
  return (__class__ || WrapperObject).__cache__;
}
Module['getCache'] = getCache;

/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant)
    @param {*=} __class__ */
function wrapPointer(ptr, __class__) {
  var cache = getCache(__class__);
  var ret = cache[ptr];
  if (ret) return ret;
  ret = Object.create((__class__ || WrapperObject).prototype);
  ret.ptr = ptr;
  return cache[ptr] = ret;
}
Module['wrapPointer'] = wrapPointer;

/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
function castObject(obj, __class__) {
  return wrapPointer(obj.ptr, __class__);
}
Module['castObject'] = castObject;

Module['NULL'] = wrapPointer(0);

/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
function destroy(obj) {
  if (!obj['__destroy__']) throw 'Error: Cannot destroy object. (Did you create it yourself?)';
  obj['__destroy__']();
  // Remove from cache, so the object can be GC'd and refs added onto it released
  delete getCache(obj.__class__)[obj.ptr];
}
Module['destroy'] = destroy;

/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
function compare(obj1, obj2) {
  return obj1.ptr === obj2.ptr;
}
Module['compare'] = compare;

/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
function getPointer(obj) {
  return obj.ptr;
}
Module['getPointer'] = getPointer;

/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
function getClass(obj) {
  return obj.__class__;
}
Module['getClass'] = getClass;

// Converts big (string or array) values into a C-style storage, in temporary space

/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
var ensureCache = {
  buffer: 0,  // the main buffer of temporary storage
  size: 0,   // the size of buffer
  pos: 0,    // the next free offset in buffer
  temps: [], // extra allocations
  needed: 0, // the total size we need next time

  prepare: function() {
    if (ensureCache.needed) {
      // clear the temps
      for (var i = 0; i < ensureCache.temps.length; i++) {
        Module['_free'](ensureCache.temps[i]);
      }
      ensureCache.temps.length = 0;
      // prepare to allocate a bigger buffer
      Module['_free'](ensureCache.buffer);
      ensureCache.buffer = 0;
      ensureCache.size += ensureCache.needed;
      // clean up
      ensureCache.needed = 0;
    }
    if (!ensureCache.buffer) { // happens first time, or when we need to grow
      ensureCache.size += 128; // heuristic, avoid many small grow events
      ensureCache.buffer = Module['_malloc'](ensureCache.size);
      assert(ensureCache.buffer);
    }
    ensureCache.pos = 0;
  },
  alloc: function(array, view) {
    assert(ensureCache.buffer);
    var bytes = view.BYTES_PER_ELEMENT;
    var len = array.length * bytes;
    len = (len + 7) & -8; // keep things aligned to 8 byte boundaries
    var ret;
    if (ensureCache.pos + len >= ensureCache.size) {
      // we failed to allocate in the buffer, ensureCache time around :(
      assert(len > 0); // null terminator, at least
      ensureCache.needed += len;
      ret = Module['_malloc'](len);
      ensureCache.temps.push(ret);
    } else {
      // we can allocate in the buffer
      ret = ensureCache.buffer + ensureCache.pos;
      ensureCache.pos += len;
    }
    return ret;
  },
  copy: function(array, view, offset) {
    offset >>>= 0;
    var bytes = view.BYTES_PER_ELEMENT;
    switch (bytes) {
      case 2: offset >>>= 1; break;
      case 4: offset >>>= 2; break;
      case 8: offset >>>= 3; break;
    }
    for (var i = 0; i < array.length; i++) {
      view[offset + i] = array[i];
    }
  },
};

/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
function ensureString(value) {
  if (typeof value === 'string') {
    var intArray = intArrayFromString(value);
    var offset = ensureCache.alloc(intArray, HEAP8);
    ensureCache.copy(intArray, HEAP8, offset);
    return offset;
  }
  return value;
}
/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
function ensureInt8(value) {
  if (typeof value === 'object') {
    var offset = ensureCache.alloc(value, HEAP8);
    ensureCache.copy(value, HEAP8, offset);
    return offset;
  }
  return value;
}
/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
function ensureInt16(value) {
  if (typeof value === 'object') {
    var offset = ensureCache.alloc(value, HEAP16);
    ensureCache.copy(value, HEAP16, offset);
    return offset;
  }
  return value;
}
/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
function ensureInt32(value) {
  if (typeof value === 'object') {
    var offset = ensureCache.alloc(value, HEAP32);
    ensureCache.copy(value, HEAP32, offset);
    return offset;
  }
  return value;
}
/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
function ensureFloat32(value) {
  if (typeof value === 'object') {
    var offset = ensureCache.alloc(value, HEAPF32);
    ensureCache.copy(value, HEAPF32, offset);
    return offset;
  }
  return value;
}
/** @suppress {duplicate} (TODO: avoid emitting this multiple times, it is redundant) */
function ensureFloat64(value) {
  if (typeof value === 'object') {
    var offset = ensureCache.alloc(value, HEAPF64);
    ensureCache.copy(value, HEAPF64, offset);
    return offset;
  }
  return value;
}


// VoidPtr
/** @suppress {undefinedVars, duplicate} @this{Object} */function VoidPtr() { throw "cannot construct a VoidPtr, no constructor in IDL" }
VoidPtr.prototype = Object.create(WrapperObject.prototype);
VoidPtr.prototype.constructor = VoidPtr;
VoidPtr.prototype.__class__ = VoidPtr;
VoidPtr.__cache__ = {};
Module['VoidPtr'] = VoidPtr;

  VoidPtr.prototype['__destroy__'] = VoidPtr.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_VoidPtr___destroy___0(self);
};
// Vec2
/** @suppress {undefinedVars, duplicate} @this{Object} */function Vec2(x, y) {
  if (x && typeof x === 'object') x = x.ptr;
  if (y && typeof y === 'object') y = y.ptr;
  if (x === undefined) { this.ptr = _emscripten_bind_Vec2_Vec2_0(); getCache(Vec2)[this.ptr] = this;return }
  if (y === undefined) { this.ptr = _emscripten_bind_Vec2_Vec2_1(x); getCache(Vec2)[this.ptr] = this;return }
  this.ptr = _emscripten_bind_Vec2_Vec2_2(x, y);
  getCache(Vec2)[this.ptr] = this;
};;
Vec2.prototype = Object.create(WrapperObject.prototype);
Vec2.prototype.constructor = Vec2;
Vec2.prototype.__class__ = Vec2;
Vec2.__cache__ = {};
Module['Vec2'] = Vec2;

Vec2.prototype['set'] = Vec2.prototype.set = /** @suppress {undefinedVars, duplicate} @this{Object} */function(x, y) {
  var self = this.ptr;
  if (x && typeof x === 'object') x = x.ptr;
  if (y && typeof y === 'object') y = y.ptr;
  _emscripten_bind_Vec2_set_2(self, x, y);
};;

Vec2.prototype['norm'] = Vec2.prototype.norm = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Vec2_norm_0(self);
};;

Vec2.prototype['norm2'] = Vec2.prototype.norm2 = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Vec2_norm2_0(self);
};;

Vec2.prototype['normalize'] = Vec2.prototype.normalize = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Vec2_normalize_0(self), Vec2);
};;

Vec2.prototype['add'] = Vec2.prototype.add = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_Vec2_add_1(self, v), Vec2);
};;

Vec2.prototype['sub'] = Vec2.prototype.sub = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_Vec2_sub_1(self, v), Vec2);
};;

Vec2.prototype['mult'] = Vec2.prototype.mult = /** @suppress {undefinedVars, duplicate} @this{Object} */function(k) {
  var self = this.ptr;
  if (k && typeof k === 'object') k = k.ptr;
  return wrapPointer(_emscripten_bind_Vec2_mult_1(self, k), Vec2);
};;

  Vec2.prototype['get_x'] = Vec2.prototype.get_x = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Vec2_get_x_0(self);
};
    Vec2.prototype['set_x'] = Vec2.prototype.set_x = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0) {
  var self = this.ptr;
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  _emscripten_bind_Vec2_set_x_1(self, arg0);
};
    Object.defineProperty(Vec2.prototype, 'x', { get: Vec2.prototype.get_x, set: Vec2.prototype.set_x });
  Vec2.prototype['get_y'] = Vec2.prototype.get_y = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Vec2_get_y_0(self);
};
    Vec2.prototype['set_y'] = Vec2.prototype.set_y = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0) {
  var self = this.ptr;
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  _emscripten_bind_Vec2_set_y_1(self, arg0);
};
    Object.defineProperty(Vec2.prototype, 'y', { get: Vec2.prototype.get_y, set: Vec2.prototype.set_y });
  Vec2.prototype['__destroy__'] = Vec2.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_Vec2___destroy___0(self);
};
// Mat2
/** @suppress {undefinedVars, duplicate} @this{Object} */function Mat2(d11, d12, d21, d22) {
  if (d11 && typeof d11 === 'object') d11 = d11.ptr;
  if (d12 && typeof d12 === 'object') d12 = d12.ptr;
  if (d21 && typeof d21 === 'object') d21 = d21.ptr;
  if (d22 && typeof d22 === 'object') d22 = d22.ptr;
  if (d11 === undefined) { this.ptr = _emscripten_bind_Mat2_Mat2_0(); getCache(Mat2)[this.ptr] = this;return }
  if (d12 === undefined) { this.ptr = _emscripten_bind_Mat2_Mat2_1(d11); getCache(Mat2)[this.ptr] = this;return }
  if (d21 === undefined) { this.ptr = _emscripten_bind_Mat2_Mat2_2(d11, d12); getCache(Mat2)[this.ptr] = this;return }
  if (d22 === undefined) { this.ptr = _emscripten_bind_Mat2_Mat2_3(d11, d12, d21); getCache(Mat2)[this.ptr] = this;return }
  this.ptr = _emscripten_bind_Mat2_Mat2_4(d11, d12, d21, d22);
  getCache(Mat2)[this.ptr] = this;
};;
Mat2.prototype = Object.create(WrapperObject.prototype);
Mat2.prototype.constructor = Mat2;
Mat2.prototype.__class__ = Mat2;
Mat2.__cache__ = {};
Module['Mat2'] = Mat2;

Mat2.prototype['zero'] = Mat2.prototype.zero = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Mat2_zero_0(self), Mat2);
};;

Mat2.prototype['identity'] = Mat2.prototype.identity = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Mat2_identity_0(self), Mat2);
};;

Mat2.prototype['at'] = Mat2.prototype.at = /** @suppress {undefinedVars, duplicate} @this{Object} */function(m, n) {
  var self = this.ptr;
  if (m && typeof m === 'object') m = m.ptr;
  if (n && typeof n === 'object') n = n.ptr;
  return _emscripten_bind_Mat2_at_2(self, m, n);
};;

Mat2.prototype['set'] = Mat2.prototype.set = /** @suppress {undefinedVars, duplicate} @this{Object} */function(m, n, val) {
  var self = this.ptr;
  if (m && typeof m === 'object') m = m.ptr;
  if (n && typeof n === 'object') n = n.ptr;
  if (val && typeof val === 'object') val = val.ptr;
  _emscripten_bind_Mat2_set_3(self, m, n, val);
};;

Mat2.prototype['add'] = Mat2.prototype.add = /** @suppress {undefinedVars, duplicate} @this{Object} */function(m) {
  var self = this.ptr;
  if (m && typeof m === 'object') m = m.ptr;
  return wrapPointer(_emscripten_bind_Mat2_add_1(self, m), Mat2);
};;

Mat2.prototype['sub'] = Mat2.prototype.sub = /** @suppress {undefinedVars, duplicate} @this{Object} */function(m) {
  var self = this.ptr;
  if (m && typeof m === 'object') m = m.ptr;
  return wrapPointer(_emscripten_bind_Mat2_sub_1(self, m), Mat2);
};;

Mat2.prototype['mult'] = Mat2.prototype.mult = /** @suppress {undefinedVars, duplicate} @this{Object} */function(m) {
  var self = this.ptr;
  if (m && typeof m === 'object') m = m.ptr;
  return wrapPointer(_emscripten_bind_Mat2_mult_1(self, m), Mat2);
};;

Mat2.prototype['determinant'] = Mat2.prototype.determinant = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Mat2_determinant_0(self);
};;

Mat2.prototype['inverse'] = Mat2.prototype.inverse = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Mat2_inverse_0(self), Mat2);
};;

Mat2.prototype['invert'] = Mat2.prototype.invert = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Mat2_invert_0(self), Mat2);
};;

Mat2.prototype['transpose'] = Mat2.prototype.transpose = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Mat2_transpose_0(self), Mat2);
};;

  Mat2.prototype['get_data'] = Mat2.prototype.get_data = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0) {
  var self = this.ptr;
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  return _emscripten_bind_Mat2_get_data_1(self, arg0);
};
    Mat2.prototype['set_data'] = Mat2.prototype.set_data = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0, arg1) {
  var self = this.ptr;
  ensureCache.prepare();
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  if (arg1 && typeof arg1 === 'object') arg1 = arg1.ptr;
  _emscripten_bind_Mat2_set_data_2(self, arg0, arg1);
};
    Object.defineProperty(Mat2.prototype, 'data', { get: Mat2.prototype.get_data, set: Mat2.prototype.set_data });
  Mat2.prototype['__destroy__'] = Mat2.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_Mat2___destroy___0(self);
};
// Mat3
/** @suppress {undefinedVars, duplicate} @this{Object} */function Mat3(d11, d12, d13, d21, d22, d23) {
  if (d11 && typeof d11 === 'object') d11 = d11.ptr;
  if (d12 && typeof d12 === 'object') d12 = d12.ptr;
  if (d13 && typeof d13 === 'object') d13 = d13.ptr;
  if (d21 && typeof d21 === 'object') d21 = d21.ptr;
  if (d22 && typeof d22 === 'object') d22 = d22.ptr;
  if (d23 && typeof d23 === 'object') d23 = d23.ptr;
  if (d11 === undefined) { this.ptr = _emscripten_bind_Mat3_Mat3_0(); getCache(Mat3)[this.ptr] = this;return }
  if (d12 === undefined) { this.ptr = _emscripten_bind_Mat3_Mat3_1(d11); getCache(Mat3)[this.ptr] = this;return }
  if (d13 === undefined) { this.ptr = _emscripten_bind_Mat3_Mat3_2(d11, d12); getCache(Mat3)[this.ptr] = this;return }
  if (d21 === undefined) { this.ptr = _emscripten_bind_Mat3_Mat3_3(d11, d12, d13); getCache(Mat3)[this.ptr] = this;return }
  if (d22 === undefined) { this.ptr = _emscripten_bind_Mat3_Mat3_4(d11, d12, d13, d21); getCache(Mat3)[this.ptr] = this;return }
  if (d23 === undefined) { this.ptr = _emscripten_bind_Mat3_Mat3_5(d11, d12, d13, d21, d22); getCache(Mat3)[this.ptr] = this;return }
  this.ptr = _emscripten_bind_Mat3_Mat3_6(d11, d12, d13, d21, d22, d23);
  getCache(Mat3)[this.ptr] = this;
};;
Mat3.prototype = Object.create(WrapperObject.prototype);
Mat3.prototype.constructor = Mat3;
Mat3.prototype.__class__ = Mat3;
Mat3.__cache__ = {};
Module['Mat3'] = Mat3;

Mat3.prototype['identity'] = Mat3.prototype.identity = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Mat3_identity_0(self), Mat3);
};;

Mat3.prototype['at'] = Mat3.prototype.at = /** @suppress {undefinedVars, duplicate} @this{Object} */function(m, n) {
  var self = this.ptr;
  if (m && typeof m === 'object') m = m.ptr;
  if (n && typeof n === 'object') n = n.ptr;
  return _emscripten_bind_Mat3_at_2(self, m, n);
};;

Mat3.prototype['set'] = Mat3.prototype.set = /** @suppress {undefinedVars, duplicate} @this{Object} */function(m, n, val) {
  var self = this.ptr;
  if (m && typeof m === 'object') m = m.ptr;
  if (n && typeof n === 'object') n = n.ptr;
  if (val && typeof val === 'object') val = val.ptr;
  _emscripten_bind_Mat3_set_3(self, m, n, val);
};;

Mat3.prototype['determinant'] = Mat3.prototype.determinant = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Mat3_determinant_0(self);
};;

Mat3.prototype['inverse'] = Mat3.prototype.inverse = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Mat3_inverse_0(self), Mat3);
};;

Mat3.prototype['invert'] = Mat3.prototype.invert = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Mat3_invert_0(self), Mat3);
};;

  Mat3.prototype['__destroy__'] = Mat3.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_Mat3___destroy___0(self);
};
// Collider
/** @suppress {undefinedVars, duplicate} @this{Object} */function Collider() { throw "cannot construct a Collider, no constructor in IDL" }
Collider.prototype = Object.create(WrapperObject.prototype);
Collider.prototype.constructor = Collider;
Collider.prototype.__class__ = Collider;
Collider.__cache__ = {};
Module['Collider'] = Collider;

Collider.prototype['origin'] = Collider.prototype.origin = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Collider_origin_0(self), Vec2);
};;

Collider.prototype['getTransform'] = Collider.prototype.getTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Collider_getTransform_0(self), Mat3);
};;

Collider.prototype['getInvTransform'] = Collider.prototype.getInvTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Collider_getInvTransform_0(self), Mat3);
};;

Collider.prototype['setTransform'] = Collider.prototype.setTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function(transform) {
  var self = this.ptr;
  if (transform && typeof transform === 'object') transform = transform.ptr;
  _emscripten_bind_Collider_setTransform_1(self, transform);
};;

Collider.prototype['setInvTransform'] = Collider.prototype.setInvTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function(invTransform) {
  var self = this.ptr;
  if (invTransform && typeof invTransform === 'object') invTransform = invTransform.ptr;
  _emscripten_bind_Collider_setInvTransform_1(self, invTransform);
};;

Collider.prototype['collider2Object'] = Collider.prototype.collider2Object = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_Collider_collider2Object_1(self, v), Vec2);
};;

Collider.prototype['object2Collider'] = Collider.prototype.object2Collider = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_Collider_object2Collider_1(self, v), Vec2);
};;

Collider.prototype['rotCollider2Object'] = Collider.prototype.rotCollider2Object = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_Collider_rotCollider2Object_1(self, v), Vec2);
};;

Collider.prototype['rotObject2Collider'] = Collider.prototype.rotObject2Collider = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_Collider_rotObject2Collider_1(self, v), Vec2);
};;

  Collider.prototype['__destroy__'] = Collider.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_Collider___destroy___0(self);
};
// Rigidbody
/** @suppress {undefinedVars, duplicate} @this{Object} */function Rigidbody() {
  this.ptr = _emscripten_bind_Rigidbody_Rigidbody_0();
  getCache(Rigidbody)[this.ptr] = this;
};;
Rigidbody.prototype = Object.create(WrapperObject.prototype);
Rigidbody.prototype.constructor = Rigidbody;
Rigidbody.prototype.__class__ = Rigidbody;
Rigidbody.__cache__ = {};
Module['Rigidbody'] = Rigidbody;

Rigidbody.prototype['getPos'] = Rigidbody.prototype.getPos = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Rigidbody_getPos_0(self), Vec2);
};;

Rigidbody.prototype['getVel'] = Rigidbody.prototype.getVel = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Rigidbody_getVel_0(self), Vec2);
};;

Rigidbody.prototype['getAcc'] = Rigidbody.prototype.getAcc = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Rigidbody_getAcc_0(self), Vec2);
};;

Rigidbody.prototype['getAngle'] = Rigidbody.prototype.getAngle = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Rigidbody_getAngle_0(self);
};;

Rigidbody.prototype['getAngVel'] = Rigidbody.prototype.getAngVel = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Rigidbody_getAngVel_0(self);
};;

Rigidbody.prototype['getMass'] = Rigidbody.prototype.getMass = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Rigidbody_getMass_0(self);
};;

Rigidbody.prototype['getInvMass'] = Rigidbody.prototype.getInvMass = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Rigidbody_getInvMass_0(self);
};;

Rigidbody.prototype['getInertia'] = Rigidbody.prototype.getInertia = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Rigidbody_getInertia_0(self);
};;

Rigidbody.prototype['getInvInertia'] = Rigidbody.prototype.getInvInertia = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Rigidbody_getInvInertia_0(self);
};;

Rigidbody.prototype['getCollider'] = Rigidbody.prototype.getCollider = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Rigidbody_getCollider_0(self), Collider);
};;

Rigidbody.prototype['getRestitution'] = Rigidbody.prototype.getRestitution = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Rigidbody_getRestitution_0(self);
};;

Rigidbody.prototype['getTransform'] = Rigidbody.prototype.getTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Rigidbody_getTransform_0(self), Mat3);
};;

Rigidbody.prototype['getInvTransform'] = Rigidbody.prototype.getInvTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Rigidbody_getInvTransform_0(self), Mat3);
};;

Rigidbody.prototype['setPos'] = Rigidbody.prototype.setPos = /** @suppress {undefinedVars, duplicate} @this{Object} */function(pos) {
  var self = this.ptr;
  if (pos && typeof pos === 'object') pos = pos.ptr;
  _emscripten_bind_Rigidbody_setPos_1(self, pos);
};;

Rigidbody.prototype['setVel'] = Rigidbody.prototype.setVel = /** @suppress {undefinedVars, duplicate} @this{Object} */function(vel) {
  var self = this.ptr;
  if (vel && typeof vel === 'object') vel = vel.ptr;
  _emscripten_bind_Rigidbody_setVel_1(self, vel);
};;

Rigidbody.prototype['setAcc'] = Rigidbody.prototype.setAcc = /** @suppress {undefinedVars, duplicate} @this{Object} */function(acc) {
  var self = this.ptr;
  if (acc && typeof acc === 'object') acc = acc.ptr;
  _emscripten_bind_Rigidbody_setAcc_1(self, acc);
};;

Rigidbody.prototype['setAngle'] = Rigidbody.prototype.setAngle = /** @suppress {undefinedVars, duplicate} @this{Object} */function(angle) {
  var self = this.ptr;
  if (angle && typeof angle === 'object') angle = angle.ptr;
  _emscripten_bind_Rigidbody_setAngle_1(self, angle);
};;

Rigidbody.prototype['setAngVel'] = Rigidbody.prototype.setAngVel = /** @suppress {undefinedVars, duplicate} @this{Object} */function(angVel) {
  var self = this.ptr;
  if (angVel && typeof angVel === 'object') angVel = angVel.ptr;
  _emscripten_bind_Rigidbody_setAngVel_1(self, angVel);
};;

Rigidbody.prototype['setMass'] = Rigidbody.prototype.setMass = /** @suppress {undefinedVars, duplicate} @this{Object} */function(mass) {
  var self = this.ptr;
  if (mass && typeof mass === 'object') mass = mass.ptr;
  _emscripten_bind_Rigidbody_setMass_1(self, mass);
};;

Rigidbody.prototype['setInvMass'] = Rigidbody.prototype.setInvMass = /** @suppress {undefinedVars, duplicate} @this{Object} */function(invMass) {
  var self = this.ptr;
  if (invMass && typeof invMass === 'object') invMass = invMass.ptr;
  _emscripten_bind_Rigidbody_setInvMass_1(self, invMass);
};;

Rigidbody.prototype['setInertia'] = Rigidbody.prototype.setInertia = /** @suppress {undefinedVars, duplicate} @this{Object} */function(inertia) {
  var self = this.ptr;
  if (inertia && typeof inertia === 'object') inertia = inertia.ptr;
  _emscripten_bind_Rigidbody_setInertia_1(self, inertia);
};;

Rigidbody.prototype['setInvInertia'] = Rigidbody.prototype.setInvInertia = /** @suppress {undefinedVars, duplicate} @this{Object} */function(invInertia) {
  var self = this.ptr;
  if (invInertia && typeof invInertia === 'object') invInertia = invInertia.ptr;
  _emscripten_bind_Rigidbody_setInvInertia_1(self, invInertia);
};;

Rigidbody.prototype['setCollider'] = Rigidbody.prototype.setCollider = /** @suppress {undefinedVars, duplicate} @this{Object} */function(collider) {
  var self = this.ptr;
  if (collider && typeof collider === 'object') collider = collider.ptr;
  _emscripten_bind_Rigidbody_setCollider_1(self, collider);
};;

Rigidbody.prototype['setRestitution'] = Rigidbody.prototype.setRestitution = /** @suppress {undefinedVars, duplicate} @this{Object} */function(restitution) {
  var self = this.ptr;
  if (restitution && typeof restitution === 'object') restitution = restitution.ptr;
  _emscripten_bind_Rigidbody_setRestitution_1(self, restitution);
};;

Rigidbody.prototype['world2Local'] = Rigidbody.prototype.world2Local = /** @suppress {undefinedVars, duplicate} @this{Object} */function(pos) {
  var self = this.ptr;
  if (pos && typeof pos === 'object') pos = pos.ptr;
  return wrapPointer(_emscripten_bind_Rigidbody_world2Local_1(self, pos), Vec2);
};;

Rigidbody.prototype['local2World'] = Rigidbody.prototype.local2World = /** @suppress {undefinedVars, duplicate} @this{Object} */function(pos) {
  var self = this.ptr;
  if (pos && typeof pos === 'object') pos = pos.ptr;
  return wrapPointer(_emscripten_bind_Rigidbody_local2World_1(self, pos), Vec2);
};;

Rigidbody.prototype['addForce'] = Rigidbody.prototype.addForce = /** @suppress {undefinedVars, duplicate} @this{Object} */function(force) {
  var self = this.ptr;
  if (force && typeof force === 'object') force = force.ptr;
  _emscripten_bind_Rigidbody_addForce_1(self, force);
};;

Rigidbody.prototype['addForceAt'] = Rigidbody.prototype.addForceAt = /** @suppress {undefinedVars, duplicate} @this{Object} */function(force, point) {
  var self = this.ptr;
  if (force && typeof force === 'object') force = force.ptr;
  if (point && typeof point === 'object') point = point.ptr;
  _emscripten_bind_Rigidbody_addForceAt_2(self, force, point);
};;

Rigidbody.prototype['addForceAtLocal'] = Rigidbody.prototype.addForceAtLocal = /** @suppress {undefinedVars, duplicate} @this{Object} */function(force, point) {
  var self = this.ptr;
  if (force && typeof force === 'object') force = force.ptr;
  if (point && typeof point === 'object') point = point.ptr;
  _emscripten_bind_Rigidbody_addForceAtLocal_2(self, force, point);
};;

Rigidbody.prototype['addTorque'] = Rigidbody.prototype.addTorque = /** @suppress {undefinedVars, duplicate} @this{Object} */function(torque) {
  var self = this.ptr;
  if (torque && typeof torque === 'object') torque = torque.ptr;
  _emscripten_bind_Rigidbody_addTorque_1(self, torque);
};;

Rigidbody.prototype['clearAccums'] = Rigidbody.prototype.clearAccums = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_Rigidbody_clearAccums_0(self);
};;

Rigidbody.prototype['step'] = Rigidbody.prototype.step = /** @suppress {undefinedVars, duplicate} @this{Object} */function(dt) {
  var self = this.ptr;
  if (dt && typeof dt === 'object') dt = dt.ptr;
  _emscripten_bind_Rigidbody_step_1(self, dt);
};;

  Rigidbody.prototype['__destroy__'] = Rigidbody.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_Rigidbody___destroy___0(self);
};
// ContactList
/** @suppress {undefinedVars, duplicate} @this{Object} */function ContactList() { throw "cannot construct a ContactList, no constructor in IDL" }
ContactList.prototype = Object.create(WrapperObject.prototype);
ContactList.prototype.constructor = ContactList;
ContactList.prototype.__class__ = ContactList;
ContactList.__cache__ = {};
Module['ContactList'] = ContactList;

// ContactGenerator
/** @suppress {undefinedVars, duplicate} @this{Object} */function ContactGenerator() { throw "cannot construct a ContactGenerator, no constructor in IDL" }
ContactGenerator.prototype = Object.create(WrapperObject.prototype);
ContactGenerator.prototype.constructor = ContactGenerator;
ContactGenerator.prototype.__class__ = ContactGenerator;
ContactGenerator.__cache__ = {};
Module['ContactGenerator'] = ContactGenerator;

ContactGenerator.prototype['generateContacts'] = ContactGenerator.prototype.generateContacts = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_ContactGenerator_generateContacts_0(self), ContactList);
};;

// CircleCollider
/** @suppress {undefinedVars, duplicate} @this{Object} */function CircleCollider(radius, body, transform) {
  if (radius && typeof radius === 'object') radius = radius.ptr;
  if (body && typeof body === 'object') body = body.ptr;
  if (transform && typeof transform === 'object') transform = transform.ptr;
  if (transform === undefined) { this.ptr = _emscripten_bind_CircleCollider_CircleCollider_2(radius, body); getCache(CircleCollider)[this.ptr] = this;return }
  this.ptr = _emscripten_bind_CircleCollider_CircleCollider_3(radius, body, transform);
  getCache(CircleCollider)[this.ptr] = this;
};;
CircleCollider.prototype = Object.create(WrapperObject.prototype);
CircleCollider.prototype.constructor = CircleCollider;
CircleCollider.prototype.__class__ = CircleCollider;
CircleCollider.__cache__ = {};
Module['CircleCollider'] = CircleCollider;

CircleCollider.prototype['origin'] = CircleCollider.prototype.origin = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_CircleCollider_origin_0(self), Vec2);
};;

CircleCollider.prototype['getTransform'] = CircleCollider.prototype.getTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_CircleCollider_getTransform_0(self), Mat3);
};;

CircleCollider.prototype['getInvTransform'] = CircleCollider.prototype.getInvTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_CircleCollider_getInvTransform_0(self), Mat3);
};;

CircleCollider.prototype['setTransform'] = CircleCollider.prototype.setTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function(transform) {
  var self = this.ptr;
  if (transform && typeof transform === 'object') transform = transform.ptr;
  _emscripten_bind_CircleCollider_setTransform_1(self, transform);
};;

CircleCollider.prototype['setInvTransform'] = CircleCollider.prototype.setInvTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function(invTransform) {
  var self = this.ptr;
  if (invTransform && typeof invTransform === 'object') invTransform = invTransform.ptr;
  _emscripten_bind_CircleCollider_setInvTransform_1(self, invTransform);
};;

CircleCollider.prototype['collider2Object'] = CircleCollider.prototype.collider2Object = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_CircleCollider_collider2Object_1(self, v), Vec2);
};;

CircleCollider.prototype['object2Collider'] = CircleCollider.prototype.object2Collider = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_CircleCollider_object2Collider_1(self, v), Vec2);
};;

CircleCollider.prototype['rotCollider2Object'] = CircleCollider.prototype.rotCollider2Object = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_CircleCollider_rotCollider2Object_1(self, v), Vec2);
};;

CircleCollider.prototype['rotObject2Collider'] = CircleCollider.prototype.rotObject2Collider = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_CircleCollider_rotObject2Collider_1(self, v), Vec2);
};;

  CircleCollider.prototype['get_radius'] = CircleCollider.prototype.get_radius = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_CircleCollider_get_radius_0(self);
};
    CircleCollider.prototype['set_radius'] = CircleCollider.prototype.set_radius = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0) {
  var self = this.ptr;
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  _emscripten_bind_CircleCollider_set_radius_1(self, arg0);
};
    Object.defineProperty(CircleCollider.prototype, 'radius', { get: CircleCollider.prototype.get_radius, set: CircleCollider.prototype.set_radius });
  CircleCollider.prototype['__destroy__'] = CircleCollider.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_CircleCollider___destroy___0(self);
};
// BoxCollider
/** @suppress {undefinedVars, duplicate} @this{Object} */function BoxCollider(halfSize, body, transform) {
  if (halfSize && typeof halfSize === 'object') halfSize = halfSize.ptr;
  if (body && typeof body === 'object') body = body.ptr;
  if (transform && typeof transform === 'object') transform = transform.ptr;
  if (transform === undefined) { this.ptr = _emscripten_bind_BoxCollider_BoxCollider_2(halfSize, body); getCache(BoxCollider)[this.ptr] = this;return }
  this.ptr = _emscripten_bind_BoxCollider_BoxCollider_3(halfSize, body, transform);
  getCache(BoxCollider)[this.ptr] = this;
};;
BoxCollider.prototype = Object.create(WrapperObject.prototype);
BoxCollider.prototype.constructor = BoxCollider;
BoxCollider.prototype.__class__ = BoxCollider;
BoxCollider.__cache__ = {};
Module['BoxCollider'] = BoxCollider;

BoxCollider.prototype['origin'] = BoxCollider.prototype.origin = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_BoxCollider_origin_0(self), Vec2);
};;

BoxCollider.prototype['getTransform'] = BoxCollider.prototype.getTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_BoxCollider_getTransform_0(self), Mat3);
};;

BoxCollider.prototype['getInvTransform'] = BoxCollider.prototype.getInvTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_BoxCollider_getInvTransform_0(self), Mat3);
};;

BoxCollider.prototype['setTransform'] = BoxCollider.prototype.setTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function(transform) {
  var self = this.ptr;
  if (transform && typeof transform === 'object') transform = transform.ptr;
  _emscripten_bind_BoxCollider_setTransform_1(self, transform);
};;

BoxCollider.prototype['setInvTransform'] = BoxCollider.prototype.setInvTransform = /** @suppress {undefinedVars, duplicate} @this{Object} */function(invTransform) {
  var self = this.ptr;
  if (invTransform && typeof invTransform === 'object') invTransform = invTransform.ptr;
  _emscripten_bind_BoxCollider_setInvTransform_1(self, invTransform);
};;

BoxCollider.prototype['collider2Object'] = BoxCollider.prototype.collider2Object = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_BoxCollider_collider2Object_1(self, v), Vec2);
};;

BoxCollider.prototype['object2Collider'] = BoxCollider.prototype.object2Collider = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_BoxCollider_object2Collider_1(self, v), Vec2);
};;

BoxCollider.prototype['rotCollider2Object'] = BoxCollider.prototype.rotCollider2Object = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_BoxCollider_rotCollider2Object_1(self, v), Vec2);
};;

BoxCollider.prototype['rotObject2Collider'] = BoxCollider.prototype.rotObject2Collider = /** @suppress {undefinedVars, duplicate} @this{Object} */function(v) {
  var self = this.ptr;
  if (v && typeof v === 'object') v = v.ptr;
  return wrapPointer(_emscripten_bind_BoxCollider_rotObject2Collider_1(self, v), Vec2);
};;

  BoxCollider.prototype['get_halfSize'] = BoxCollider.prototype.get_halfSize = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_BoxCollider_get_halfSize_0(self), Vec2);
};
    BoxCollider.prototype['set_halfSize'] = BoxCollider.prototype.set_halfSize = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0) {
  var self = this.ptr;
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  _emscripten_bind_BoxCollider_set_halfSize_1(self, arg0);
};
    Object.defineProperty(BoxCollider.prototype, 'halfSize', { get: BoxCollider.prototype.get_halfSize, set: BoxCollider.prototype.set_halfSize });
  BoxCollider.prototype['__destroy__'] = BoxCollider.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_BoxCollider___destroy___0(self);
};
// IntersectionDetector
/** @suppress {undefinedVars, duplicate} @this{Object} */function IntersectionDetector() { throw "cannot construct a IntersectionDetector, no constructor in IDL" }
IntersectionDetector.prototype = Object.create(WrapperObject.prototype);
IntersectionDetector.prototype.constructor = IntersectionDetector;
IntersectionDetector.prototype.__class__ = IntersectionDetector;
IntersectionDetector.__cache__ = {};
Module['IntersectionDetector'] = IntersectionDetector;

IntersectionDetector.prototype['circleCircle'] = IntersectionDetector.prototype.circleCircle = /** @suppress {undefinedVars, duplicate} @this{Object} */function(c1, c2) {
  var self = this.ptr;
  if (c1 && typeof c1 === 'object') c1 = c1.ptr;
  if (c2 && typeof c2 === 'object') c2 = c2.ptr;
  return !!(_emscripten_bind_IntersectionDetector_circleCircle_2(self, c1, c2));
};;

IntersectionDetector.prototype['boxCircle'] = IntersectionDetector.prototype.boxCircle = /** @suppress {undefinedVars, duplicate} @this{Object} */function(bc, cc) {
  var self = this.ptr;
  if (bc && typeof bc === 'object') bc = bc.ptr;
  if (cc && typeof cc === 'object') cc = cc.ptr;
  return !!(_emscripten_bind_IntersectionDetector_boxCircle_2(self, bc, cc));
};;

IntersectionDetector.prototype['boxBox'] = IntersectionDetector.prototype.boxBox = /** @suppress {undefinedVars, duplicate} @this{Object} */function(b1, b2) {
  var self = this.ptr;
  if (b1 && typeof b1 === 'object') b1 = b1.ptr;
  if (b2 && typeof b2 === 'object') b2 = b2.ptr;
  return !!(_emscripten_bind_IntersectionDetector_boxBox_2(self, b1, b2));
};;

// ForceGenerator
/** @suppress {undefinedVars, duplicate} @this{Object} */function ForceGenerator() { throw "cannot construct a ForceGenerator, no constructor in IDL" }
ForceGenerator.prototype = Object.create(WrapperObject.prototype);
ForceGenerator.prototype.constructor = ForceGenerator;
ForceGenerator.prototype.__class__ = ForceGenerator;
ForceGenerator.__cache__ = {};
Module['ForceGenerator'] = ForceGenerator;

ForceGenerator.prototype['updateForce'] = ForceGenerator.prototype.updateForce = /** @suppress {undefinedVars, duplicate} @this{Object} */function(body, dt) {
  var self = this.ptr;
  if (body && typeof body === 'object') body = body.ptr;
  if (dt && typeof dt === 'object') dt = dt.ptr;
  _emscripten_bind_ForceGenerator_updateForce_2(self, body, dt);
};;

// Gravity
/** @suppress {undefinedVars, duplicate} @this{Object} */function Gravity(gravity) {
  if (gravity && typeof gravity === 'object') gravity = gravity.ptr;
  this.ptr = _emscripten_bind_Gravity_Gravity_1(gravity);
  getCache(Gravity)[this.ptr] = this;
};;
Gravity.prototype = Object.create(WrapperObject.prototype);
Gravity.prototype.constructor = Gravity;
Gravity.prototype.__class__ = Gravity;
Gravity.__cache__ = {};
Module['Gravity'] = Gravity;

Gravity.prototype['updateForce'] = Gravity.prototype.updateForce = /** @suppress {undefinedVars, duplicate} @this{Object} */function(body, dt) {
  var self = this.ptr;
  if (body && typeof body === 'object') body = body.ptr;
  if (dt && typeof dt === 'object') dt = dt.ptr;
  _emscripten_bind_Gravity_updateForce_2(self, body, dt);
};;

  Gravity.prototype['__destroy__'] = Gravity.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_Gravity___destroy___0(self);
};
// Spring
/** @suppress {undefinedVars, duplicate} @this{Object} */function Spring(end, anchor, endAnchor, k, length) {
  if (end && typeof end === 'object') end = end.ptr;
  if (anchor && typeof anchor === 'object') anchor = anchor.ptr;
  if (endAnchor && typeof endAnchor === 'object') endAnchor = endAnchor.ptr;
  if (k && typeof k === 'object') k = k.ptr;
  if (length && typeof length === 'object') length = length.ptr;
  this.ptr = _emscripten_bind_Spring_Spring_5(end, anchor, endAnchor, k, length);
  getCache(Spring)[this.ptr] = this;
};;
Spring.prototype = Object.create(WrapperObject.prototype);
Spring.prototype.constructor = Spring;
Spring.prototype.__class__ = Spring;
Spring.__cache__ = {};
Module['Spring'] = Spring;

Spring.prototype['getEnd'] = Spring.prototype.getEnd = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Spring_getEnd_0(self), Rigidbody);
};;

Spring.prototype['setEnd'] = Spring.prototype.setEnd = /** @suppress {undefinedVars, duplicate} @this{Object} */function(body) {
  var self = this.ptr;
  if (body && typeof body === 'object') body = body.ptr;
  _emscripten_bind_Spring_setEnd_1(self, body);
};;

Spring.prototype['updateForce'] = Spring.prototype.updateForce = /** @suppress {undefinedVars, duplicate} @this{Object} */function(body, dt) {
  var self = this.ptr;
  if (body && typeof body === 'object') body = body.ptr;
  if (dt && typeof dt === 'object') dt = dt.ptr;
  _emscripten_bind_Spring_updateForce_2(self, body, dt);
};;

  Spring.prototype['get_k'] = Spring.prototype.get_k = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Spring_get_k_0(self);
};
    Spring.prototype['set_k'] = Spring.prototype.set_k = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0) {
  var self = this.ptr;
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  _emscripten_bind_Spring_set_k_1(self, arg0);
};
    Object.defineProperty(Spring.prototype, 'k', { get: Spring.prototype.get_k, set: Spring.prototype.set_k });
  Spring.prototype['get_length'] = Spring.prototype.get_length = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Spring_get_length_0(self);
};
    Spring.prototype['set_length'] = Spring.prototype.set_length = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0) {
  var self = this.ptr;
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  _emscripten_bind_Spring_set_length_1(self, arg0);
};
    Object.defineProperty(Spring.prototype, 'length', { get: Spring.prototype.get_length, set: Spring.prototype.set_length });
  Spring.prototype['__destroy__'] = Spring.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_Spring___destroy___0(self);
};
// World
/** @suppress {undefinedVars, duplicate} @this{Object} */function World() {
  this.ptr = _emscripten_bind_World_World_0();
  getCache(World)[this.ptr] = this;
};;
World.prototype = Object.create(WrapperObject.prototype);
World.prototype.constructor = World;
World.prototype.__class__ = World;
World.__cache__ = {};
Module['World'] = World;

World.prototype['addBody'] = World.prototype.addBody = /** @suppress {undefinedVars, duplicate} @this{Object} */function(body, collider) {
  var self = this.ptr;
  if (body && typeof body === 'object') body = body.ptr;
  if (collider && typeof collider === 'object') collider = collider.ptr;
  _emscripten_bind_World_addBody_2(self, body, collider);
};;

World.prototype['removeBody'] = World.prototype.removeBody = /** @suppress {undefinedVars, duplicate} @this{Object} */function(body, collider) {
  var self = this.ptr;
  if (body && typeof body === 'object') body = body.ptr;
  if (collider && typeof collider === 'object') collider = collider.ptr;
  _emscripten_bind_World_removeBody_2(self, body, collider);
};;

World.prototype['addFGen'] = World.prototype.addFGen = /** @suppress {undefinedVars, duplicate} @this{Object} */function(fgen, body) {
  var self = this.ptr;
  if (fgen && typeof fgen === 'object') fgen = fgen.ptr;
  if (body && typeof body === 'object') body = body.ptr;
  _emscripten_bind_World_addFGen_2(self, fgen, body);
};;

World.prototype['removeFGen'] = World.prototype.removeFGen = /** @suppress {undefinedVars, duplicate} @this{Object} */function(fgen, body) {
  var self = this.ptr;
  if (fgen && typeof fgen === 'object') fgen = fgen.ptr;
  if (body && typeof body === 'object') body = body.ptr;
  _emscripten_bind_World_removeFGen_2(self, fgen, body);
};;

World.prototype['addContactGen'] = World.prototype.addContactGen = /** @suppress {undefinedVars, duplicate} @this{Object} */function(cgen) {
  var self = this.ptr;
  if (cgen && typeof cgen === 'object') cgen = cgen.ptr;
  _emscripten_bind_World_addContactGen_1(self, cgen);
};;

World.prototype['removeContactGen'] = World.prototype.removeContactGen = /** @suppress {undefinedVars, duplicate} @this{Object} */function(cgen) {
  var self = this.ptr;
  if (cgen && typeof cgen === 'object') cgen = cgen.ptr;
  _emscripten_bind_World_removeContactGen_1(self, cgen);
};;

World.prototype['step'] = World.prototype.step = /** @suppress {undefinedVars, duplicate} @this{Object} */function(dt) {
  var self = this.ptr;
  if (dt && typeof dt === 'object') dt = dt.ptr;
  _emscripten_bind_World_step_1(self, dt);
};;

  World.prototype['__destroy__'] = World.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_World___destroy___0(self);
};
// Particle
/** @suppress {undefinedVars, duplicate} @this{Object} */function Particle(mass, damping) {
  if (mass && typeof mass === 'object') mass = mass.ptr;
  if (damping && typeof damping === 'object') damping = damping.ptr;
  if (mass === undefined) { this.ptr = _emscripten_bind_Particle_Particle_0(); getCache(Particle)[this.ptr] = this;return }
  if (damping === undefined) { this.ptr = _emscripten_bind_Particle_Particle_1(mass); getCache(Particle)[this.ptr] = this;return }
  this.ptr = _emscripten_bind_Particle_Particle_2(mass, damping);
  getCache(Particle)[this.ptr] = this;
};;
Particle.prototype = Object.create(WrapperObject.prototype);
Particle.prototype.constructor = Particle;
Particle.prototype.__class__ = Particle;
Particle.__cache__ = {};
Module['Particle'] = Particle;

Particle.prototype['getPos'] = Particle.prototype.getPos = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Particle_getPos_0(self), Vec2);
};;

Particle.prototype['getVel'] = Particle.prototype.getVel = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Particle_getVel_0(self), Vec2);
};;

Particle.prototype['getAcc'] = Particle.prototype.getAcc = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_Particle_getAcc_0(self), Vec2);
};;

Particle.prototype['getMass'] = Particle.prototype.getMass = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Particle_getMass_0(self);
};;

Particle.prototype['getInvMass'] = Particle.prototype.getInvMass = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Particle_getInvMass_0(self);
};;

Particle.prototype['getDamping'] = Particle.prototype.getDamping = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_Particle_getDamping_0(self);
};;

Particle.prototype['setPos'] = Particle.prototype.setPos = /** @suppress {undefinedVars, duplicate} @this{Object} */function(pos) {
  var self = this.ptr;
  if (pos && typeof pos === 'object') pos = pos.ptr;
  _emscripten_bind_Particle_setPos_1(self, pos);
};;

Particle.prototype['setVel'] = Particle.prototype.setVel = /** @suppress {undefinedVars, duplicate} @this{Object} */function(vel) {
  var self = this.ptr;
  if (vel && typeof vel === 'object') vel = vel.ptr;
  _emscripten_bind_Particle_setVel_1(self, vel);
};;

Particle.prototype['setAcc'] = Particle.prototype.setAcc = /** @suppress {undefinedVars, duplicate} @this{Object} */function(acc) {
  var self = this.ptr;
  if (acc && typeof acc === 'object') acc = acc.ptr;
  _emscripten_bind_Particle_setAcc_1(self, acc);
};;

Particle.prototype['setMass'] = Particle.prototype.setMass = /** @suppress {undefinedVars, duplicate} @this{Object} */function(mass) {
  var self = this.ptr;
  if (mass && typeof mass === 'object') mass = mass.ptr;
  _emscripten_bind_Particle_setMass_1(self, mass);
};;

Particle.prototype['setInvMass'] = Particle.prototype.setInvMass = /** @suppress {undefinedVars, duplicate} @this{Object} */function(mass) {
  var self = this.ptr;
  if (mass && typeof mass === 'object') mass = mass.ptr;
  _emscripten_bind_Particle_setInvMass_1(self, mass);
};;

Particle.prototype['setDamping'] = Particle.prototype.setDamping = /** @suppress {undefinedVars, duplicate} @this{Object} */function(damping) {
  var self = this.ptr;
  if (damping && typeof damping === 'object') damping = damping.ptr;
  _emscripten_bind_Particle_setDamping_1(self, damping);
};;

Particle.prototype['addForce'] = Particle.prototype.addForce = /** @suppress {undefinedVars, duplicate} @this{Object} */function(force) {
  var self = this.ptr;
  if (force && typeof force === 'object') force = force.ptr;
  _emscripten_bind_Particle_addForce_1(self, force);
};;

Particle.prototype['addImpulse'] = Particle.prototype.addImpulse = /** @suppress {undefinedVars, duplicate} @this{Object} */function(impulse) {
  var self = this.ptr;
  if (impulse && typeof impulse === 'object') impulse = impulse.ptr;
  _emscripten_bind_Particle_addImpulse_1(self, impulse);
};;

Particle.prototype['clearForceAccum'] = Particle.prototype.clearForceAccum = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_Particle_clearForceAccum_0(self);
};;

Particle.prototype['setStatic'] = Particle.prototype.setStatic = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_Particle_setStatic_0(self);
};;

Particle.prototype['isStatic'] = Particle.prototype.isStatic = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return !!(_emscripten_bind_Particle_isStatic_0(self));
};;

Particle.prototype['step'] = Particle.prototype.step = /** @suppress {undefinedVars, duplicate} @this{Object} */function(dt) {
  var self = this.ptr;
  if (dt && typeof dt === 'object') dt = dt.ptr;
  _emscripten_bind_Particle_step_1(self, dt);
};;

  Particle.prototype['__destroy__'] = Particle.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_Particle___destroy___0(self);
};
// ParticleForceGenerator
/** @suppress {undefinedVars, duplicate} @this{Object} */function ParticleForceGenerator() { throw "cannot construct a ParticleForceGenerator, no constructor in IDL" }
ParticleForceGenerator.prototype = Object.create(WrapperObject.prototype);
ParticleForceGenerator.prototype.constructor = ParticleForceGenerator;
ParticleForceGenerator.prototype.__class__ = ParticleForceGenerator;
ParticleForceGenerator.__cache__ = {};
Module['ParticleForceGenerator'] = ParticleForceGenerator;

ParticleForceGenerator.prototype['updateForce'] = ParticleForceGenerator.prototype.updateForce = /** @suppress {undefinedVars, duplicate} @this{Object} */function(particle, dt) {
  var self = this.ptr;
  if (particle && typeof particle === 'object') particle = particle.ptr;
  if (dt && typeof dt === 'object') dt = dt.ptr;
  _emscripten_bind_ParticleForceGenerator_updateForce_2(self, particle, dt);
};;

// ParticleGravity
/** @suppress {undefinedVars, duplicate} @this{Object} */function ParticleGravity(gravity) {
  if (gravity && typeof gravity === 'object') gravity = gravity.ptr;
  this.ptr = _emscripten_bind_ParticleGravity_ParticleGravity_1(gravity);
  getCache(ParticleGravity)[this.ptr] = this;
};;
ParticleGravity.prototype = Object.create(WrapperObject.prototype);
ParticleGravity.prototype.constructor = ParticleGravity;
ParticleGravity.prototype.__class__ = ParticleGravity;
ParticleGravity.__cache__ = {};
Module['ParticleGravity'] = ParticleGravity;

ParticleGravity.prototype['updateForce'] = ParticleGravity.prototype.updateForce = /** @suppress {undefinedVars, duplicate} @this{Object} */function(particle, dt) {
  var self = this.ptr;
  if (particle && typeof particle === 'object') particle = particle.ptr;
  if (dt && typeof dt === 'object') dt = dt.ptr;
  _emscripten_bind_ParticleGravity_updateForce_2(self, particle, dt);
};;

  ParticleGravity.prototype['__destroy__'] = ParticleGravity.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_ParticleGravity___destroy___0(self);
};
// ParticleDrag
/** @suppress {undefinedVars, duplicate} @this{Object} */function ParticleDrag(k1, k2) {
  if (k1 && typeof k1 === 'object') k1 = k1.ptr;
  if (k2 && typeof k2 === 'object') k2 = k2.ptr;
  this.ptr = _emscripten_bind_ParticleDrag_ParticleDrag_2(k1, k2);
  getCache(ParticleDrag)[this.ptr] = this;
};;
ParticleDrag.prototype = Object.create(WrapperObject.prototype);
ParticleDrag.prototype.constructor = ParticleDrag;
ParticleDrag.prototype.__class__ = ParticleDrag;
ParticleDrag.__cache__ = {};
Module['ParticleDrag'] = ParticleDrag;

ParticleDrag.prototype['updateForce'] = ParticleDrag.prototype.updateForce = /** @suppress {undefinedVars, duplicate} @this{Object} */function(particle, dt) {
  var self = this.ptr;
  if (particle && typeof particle === 'object') particle = particle.ptr;
  if (dt && typeof dt === 'object') dt = dt.ptr;
  _emscripten_bind_ParticleDrag_updateForce_2(self, particle, dt);
};;

  ParticleDrag.prototype['__destroy__'] = ParticleDrag.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_ParticleDrag___destroy___0(self);
};
// ParticleSpring
/** @suppress {undefinedVars, duplicate} @this{Object} */function ParticleSpring(end, k, length) {
  if (end && typeof end === 'object') end = end.ptr;
  if (k && typeof k === 'object') k = k.ptr;
  if (length && typeof length === 'object') length = length.ptr;
  this.ptr = _emscripten_bind_ParticleSpring_ParticleSpring_3(end, k, length);
  getCache(ParticleSpring)[this.ptr] = this;
};;
ParticleSpring.prototype = Object.create(WrapperObject.prototype);
ParticleSpring.prototype.constructor = ParticleSpring;
ParticleSpring.prototype.__class__ = ParticleSpring;
ParticleSpring.__cache__ = {};
Module['ParticleSpring'] = ParticleSpring;

ParticleSpring.prototype['setEnd'] = ParticleSpring.prototype.setEnd = /** @suppress {undefinedVars, duplicate} @this{Object} */function(particle) {
  var self = this.ptr;
  if (particle && typeof particle === 'object') particle = particle.ptr;
  _emscripten_bind_ParticleSpring_setEnd_1(self, particle);
};;

ParticleSpring.prototype['getEnd'] = ParticleSpring.prototype.getEnd = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_ParticleSpring_getEnd_0(self), Particle);
};;

ParticleSpring.prototype['updateForce'] = ParticleSpring.prototype.updateForce = /** @suppress {undefinedVars, duplicate} @this{Object} */function(particle, dt) {
  var self = this.ptr;
  if (particle && typeof particle === 'object') particle = particle.ptr;
  if (dt && typeof dt === 'object') dt = dt.ptr;
  _emscripten_bind_ParticleSpring_updateForce_2(self, particle, dt);
};;

  ParticleSpring.prototype['get_k'] = ParticleSpring.prototype.get_k = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_ParticleSpring_get_k_0(self);
};
    ParticleSpring.prototype['set_k'] = ParticleSpring.prototype.set_k = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0) {
  var self = this.ptr;
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  _emscripten_bind_ParticleSpring_set_k_1(self, arg0);
};
    Object.defineProperty(ParticleSpring.prototype, 'k', { get: ParticleSpring.prototype.get_k, set: ParticleSpring.prototype.set_k });
  ParticleSpring.prototype['get_length'] = ParticleSpring.prototype.get_length = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_ParticleSpring_get_length_0(self);
};
    ParticleSpring.prototype['set_length'] = ParticleSpring.prototype.set_length = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0) {
  var self = this.ptr;
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  _emscripten_bind_ParticleSpring_set_length_1(self, arg0);
};
    Object.defineProperty(ParticleSpring.prototype, 'length', { get: ParticleSpring.prototype.get_length, set: ParticleSpring.prototype.set_length });
  ParticleSpring.prototype['__destroy__'] = ParticleSpring.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_ParticleSpring___destroy___0(self);
};
// PContactList
/** @suppress {undefinedVars, duplicate} @this{Object} */function PContactList() { throw "cannot construct a PContactList, no constructor in IDL" }
PContactList.prototype = Object.create(WrapperObject.prototype);
PContactList.prototype.constructor = PContactList;
PContactList.prototype.__class__ = PContactList;
PContactList.__cache__ = {};
Module['PContactList'] = PContactList;

// ParticleContactGenerator
/** @suppress {undefinedVars, duplicate} @this{Object} */function ParticleContactGenerator() { throw "cannot construct a ParticleContactGenerator, no constructor in IDL" }
ParticleContactGenerator.prototype = Object.create(WrapperObject.prototype);
ParticleContactGenerator.prototype.constructor = ParticleContactGenerator;
ParticleContactGenerator.prototype.__class__ = ParticleContactGenerator;
ParticleContactGenerator.__cache__ = {};
Module['ParticleContactGenerator'] = ParticleContactGenerator;

ParticleContactGenerator.prototype['generateContacts'] = ParticleContactGenerator.prototype.generateContacts = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_ParticleContactGenerator_generateContacts_0(self), PContactList);
};;

// ParticleLink
/** @suppress {undefinedVars, duplicate} @this{Object} */function ParticleLink() { throw "cannot construct a ParticleLink, no constructor in IDL" }
ParticleLink.prototype = Object.create(WrapperObject.prototype);
ParticleLink.prototype.constructor = ParticleLink;
ParticleLink.prototype.__class__ = ParticleLink;
ParticleLink.__cache__ = {};
Module['ParticleLink'] = ParticleLink;

ParticleLink.prototype['setParticle'] = ParticleLink.prototype.setParticle = /** @suppress {undefinedVars, duplicate} @this{Object} */function(index, particle) {
  var self = this.ptr;
  if (index && typeof index === 'object') index = index.ptr;
  if (particle && typeof particle === 'object') particle = particle.ptr;
  _emscripten_bind_ParticleLink_setParticle_2(self, index, particle);
};;

ParticleLink.prototype['getParticle'] = ParticleLink.prototype.getParticle = /** @suppress {undefinedVars, duplicate} @this{Object} */function(index) {
  var self = this.ptr;
  if (index && typeof index === 'object') index = index.ptr;
  return wrapPointer(_emscripten_bind_ParticleLink_getParticle_1(self, index), Particle);
};;

ParticleLink.prototype['generateContacts'] = ParticleLink.prototype.generateContacts = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_ParticleLink_generateContacts_0(self), PContactList);
};;

// ParticleCable
/** @suppress {undefinedVars, duplicate} @this{Object} */function ParticleCable(length, restitution) {
  if (length && typeof length === 'object') length = length.ptr;
  if (restitution && typeof restitution === 'object') restitution = restitution.ptr;
  this.ptr = _emscripten_bind_ParticleCable_ParticleCable_2(length, restitution);
  getCache(ParticleCable)[this.ptr] = this;
};;
ParticleCable.prototype = Object.create(WrapperObject.prototype);
ParticleCable.prototype.constructor = ParticleCable;
ParticleCable.prototype.__class__ = ParticleCable;
ParticleCable.__cache__ = {};
Module['ParticleCable'] = ParticleCable;

ParticleCable.prototype['setParticle'] = ParticleCable.prototype.setParticle = /** @suppress {undefinedVars, duplicate} @this{Object} */function(index, particle) {
  var self = this.ptr;
  if (index && typeof index === 'object') index = index.ptr;
  if (particle && typeof particle === 'object') particle = particle.ptr;
  _emscripten_bind_ParticleCable_setParticle_2(self, index, particle);
};;

ParticleCable.prototype['getParticle'] = ParticleCable.prototype.getParticle = /** @suppress {undefinedVars, duplicate} @this{Object} */function(index) {
  var self = this.ptr;
  if (index && typeof index === 'object') index = index.ptr;
  return wrapPointer(_emscripten_bind_ParticleCable_getParticle_1(self, index), Particle);
};;

ParticleCable.prototype['generateContacts'] = ParticleCable.prototype.generateContacts = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_ParticleCable_generateContacts_0(self), PContactList);
};;

  ParticleCable.prototype['get_length'] = ParticleCable.prototype.get_length = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_ParticleCable_get_length_0(self);
};
    ParticleCable.prototype['set_length'] = ParticleCable.prototype.set_length = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0) {
  var self = this.ptr;
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  _emscripten_bind_ParticleCable_set_length_1(self, arg0);
};
    Object.defineProperty(ParticleCable.prototype, 'length', { get: ParticleCable.prototype.get_length, set: ParticleCable.prototype.set_length });
  ParticleCable.prototype['get_restitution'] = ParticleCable.prototype.get_restitution = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_ParticleCable_get_restitution_0(self);
};
    ParticleCable.prototype['set_restitution'] = ParticleCable.prototype.set_restitution = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0) {
  var self = this.ptr;
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  _emscripten_bind_ParticleCable_set_restitution_1(self, arg0);
};
    Object.defineProperty(ParticleCable.prototype, 'restitution', { get: ParticleCable.prototype.get_restitution, set: ParticleCable.prototype.set_restitution });
  ParticleCable.prototype['__destroy__'] = ParticleCable.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_ParticleCable___destroy___0(self);
};
// ParticleRod
/** @suppress {undefinedVars, duplicate} @this{Object} */function ParticleRod(length) {
  if (length && typeof length === 'object') length = length.ptr;
  this.ptr = _emscripten_bind_ParticleRod_ParticleRod_1(length);
  getCache(ParticleRod)[this.ptr] = this;
};;
ParticleRod.prototype = Object.create(WrapperObject.prototype);
ParticleRod.prototype.constructor = ParticleRod;
ParticleRod.prototype.__class__ = ParticleRod;
ParticleRod.__cache__ = {};
Module['ParticleRod'] = ParticleRod;

ParticleRod.prototype['setParticle'] = ParticleRod.prototype.setParticle = /** @suppress {undefinedVars, duplicate} @this{Object} */function(index, particle) {
  var self = this.ptr;
  if (index && typeof index === 'object') index = index.ptr;
  if (particle && typeof particle === 'object') particle = particle.ptr;
  _emscripten_bind_ParticleRod_setParticle_2(self, index, particle);
};;

ParticleRod.prototype['getParticle'] = ParticleRod.prototype.getParticle = /** @suppress {undefinedVars, duplicate} @this{Object} */function(index) {
  var self = this.ptr;
  if (index && typeof index === 'object') index = index.ptr;
  return wrapPointer(_emscripten_bind_ParticleRod_getParticle_1(self, index), Particle);
};;

ParticleRod.prototype['generateContacts'] = ParticleRod.prototype.generateContacts = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_ParticleRod_generateContacts_0(self), PContactList);
};;

  ParticleRod.prototype['get_length'] = ParticleRod.prototype.get_length = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return _emscripten_bind_ParticleRod_get_length_0(self);
};
    ParticleRod.prototype['set_length'] = ParticleRod.prototype.set_length = /** @suppress {undefinedVars, duplicate} @this{Object} */function(arg0) {
  var self = this.ptr;
  if (arg0 && typeof arg0 === 'object') arg0 = arg0.ptr;
  _emscripten_bind_ParticleRod_set_length_1(self, arg0);
};
    Object.defineProperty(ParticleRod.prototype, 'length', { get: ParticleRod.prototype.get_length, set: ParticleRod.prototype.set_length });
  ParticleRod.prototype['__destroy__'] = ParticleRod.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_ParticleRod___destroy___0(self);
};
// ParticleWorld
/** @suppress {undefinedVars, duplicate} @this{Object} */function ParticleWorld(iterations) {
  if (iterations && typeof iterations === 'object') iterations = iterations.ptr;
  if (iterations === undefined) { this.ptr = _emscripten_bind_ParticleWorld_ParticleWorld_0(); getCache(ParticleWorld)[this.ptr] = this;return }
  this.ptr = _emscripten_bind_ParticleWorld_ParticleWorld_1(iterations);
  getCache(ParticleWorld)[this.ptr] = this;
};;
ParticleWorld.prototype = Object.create(WrapperObject.prototype);
ParticleWorld.prototype.constructor = ParticleWorld;
ParticleWorld.prototype.__class__ = ParticleWorld;
ParticleWorld.__cache__ = {};
Module['ParticleWorld'] = ParticleWorld;

ParticleWorld.prototype['addParticle'] = ParticleWorld.prototype.addParticle = /** @suppress {undefinedVars, duplicate} @this{Object} */function(particle) {
  var self = this.ptr;
  if (particle && typeof particle === 'object') particle = particle.ptr;
  _emscripten_bind_ParticleWorld_addParticle_1(self, particle);
};;

ParticleWorld.prototype['removeParticle'] = ParticleWorld.prototype.removeParticle = /** @suppress {undefinedVars, duplicate} @this{Object} */function(particle) {
  var self = this.ptr;
  if (particle && typeof particle === 'object') particle = particle.ptr;
  _emscripten_bind_ParticleWorld_removeParticle_1(self, particle);
};;

ParticleWorld.prototype['addPFGen'] = ParticleWorld.prototype.addPFGen = /** @suppress {undefinedVars, duplicate} @this{Object} */function(particle, fgen) {
  var self = this.ptr;
  if (particle && typeof particle === 'object') particle = particle.ptr;
  if (fgen && typeof fgen === 'object') fgen = fgen.ptr;
  _emscripten_bind_ParticleWorld_addPFGen_2(self, particle, fgen);
};;

ParticleWorld.prototype['removePFGen'] = ParticleWorld.prototype.removePFGen = /** @suppress {undefinedVars, duplicate} @this{Object} */function(particle, fgen) {
  var self = this.ptr;
  if (particle && typeof particle === 'object') particle = particle.ptr;
  if (fgen && typeof fgen === 'object') fgen = fgen.ptr;
  _emscripten_bind_ParticleWorld_removePFGen_2(self, particle, fgen);
};;

ParticleWorld.prototype['addPContactGenerator'] = ParticleWorld.prototype.addPContactGenerator = /** @suppress {undefinedVars, duplicate} @this{Object} */function(pcg) {
  var self = this.ptr;
  if (pcg && typeof pcg === 'object') pcg = pcg.ptr;
  _emscripten_bind_ParticleWorld_addPContactGenerator_1(self, pcg);
};;

ParticleWorld.prototype['removePContactGenerator'] = ParticleWorld.prototype.removePContactGenerator = /** @suppress {undefinedVars, duplicate} @this{Object} */function(pcg) {
  var self = this.ptr;
  if (pcg && typeof pcg === 'object') pcg = pcg.ptr;
  _emscripten_bind_ParticleWorld_removePContactGenerator_1(self, pcg);
};;

ParticleWorld.prototype['generateContacts'] = ParticleWorld.prototype.generateContacts = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  return wrapPointer(_emscripten_bind_ParticleWorld_generateContacts_0(self), PContactList);
};;

ParticleWorld.prototype['step'] = ParticleWorld.prototype.step = /** @suppress {undefinedVars, duplicate} @this{Object} */function(dt) {
  var self = this.ptr;
  if (dt && typeof dt === 'object') dt = dt.ptr;
  _emscripten_bind_ParticleWorld_step_1(self, dt);
};;

  ParticleWorld.prototype['__destroy__'] = ParticleWorld.prototype.__destroy__ = /** @suppress {undefinedVars, duplicate} @this{Object} */function() {
  var self = this.ptr;
  _emscripten_bind_ParticleWorld___destroy___0(self);
};

  return ephys.ready
}
);
})();
if (typeof exports === 'object' && typeof module === 'object')
  module.exports = ephys;
else if (typeof define === 'function' && define['amd'])
  define([], function() { return ephys; });
else if (typeof exports === 'object')
  exports["ephys"] = ephys;
