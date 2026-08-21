(() => {
  'use strict';

  const loaderElement = document.currentScript;
  const assetBase = new URL('.', loaderElement?.src || document.baseURI);
  const loadState = {
    requestedBackend: 'simd',
    backend: 'loading',
    version: '0.20.0',
    simd: false,
    fallback: false,
    fallbackReason: null,
    selectedAsset: null,
    attempts: [],
  };
  globalThis.BiteRapierLoadState = loadState;
  const forcedSimdInitFailureForQa = new URLSearchParams(globalThis.location?.search || '').has('qaForceRapierCompat');

  const errorText = (error) => {
    const value = String(error?.message || error || '未知初始化错误').replace(/\s+/g, ' ').trim();
    return value.length > 240 ? `${value.slice(0, 237)}...` : value;
  };

  const loadClassicScript = (filename) => new Promise((resolve, reject) => {
    const element = document.createElement('script');
    element.src = new URL(filename, assetBase).href;
    element.async = false;
    element.dataset.rapierBackendAsset = filename;
    element.onload = () => resolve(element);
    element.onerror = () => reject(new Error(`无法加载本地物理引擎文件 ${filename}`));
    document.head.append(element);
  });

  const loadBackend = async (backend, filename) => {
    globalThis.BiteRapier = null;
    const attempt = { backend, asset: filename, ok: false, error: null };
    loadState.attempts.push(attempt);
    try {
      await loadClassicScript(filename);
      const api = globalThis.BiteRapier;
      if (!api?.init || !api?.World) throw new Error(`${filename} 未导出完整 Rapier API`);
      if (backend === 'simd' && forcedSimdInitFailureForQa) throw new Error('本地 QA 强制验证 SIMD 初始化失败后的兼容回退');
      await api.init();
      const version = typeof api.version === 'function' ? api.version() : null;
      if (version !== loadState.version) throw new Error(`Rapier 版本不匹配：期望 ${loadState.version}，实际 ${version || '未知'}`);
      attempt.ok = true;
      loadState.backend = backend;
      loadState.simd = backend === 'simd';
      loadState.selectedAsset = filename;
      return api;
    } catch (error) {
      attempt.error = errorText(error);
      globalThis.BiteRapier = null;
      throw error;
    }
  };

  globalThis.BiteRapierReady = (async () => {
    try {
      return await loadBackend('simd', 'rapier2d-simd-compat-0.20.0.global.js');
    } catch (simdError) {
      loadState.fallback = true;
      loadState.fallbackReason = errorText(simdError);
      try {
        return await loadBackend('compat', 'rapier2d-compat-0.20.0.global.js');
      } catch (compatError) {
        loadState.backend = 'unavailable';
        const combined = new Error(`SIMD 初始化失败：${loadState.fallbackReason}；兼容后端也失败：${errorText(compatError)}`);
        combined.cause = compatError;
        throw combined;
      }
    }
  })();
})();
