"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import time
import requests
from requests.exceptions import (SSLError, RequestException, HTTPError)
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.models.helpers import is_bundle_version_compatible
from openpilot.cereal import custom

USBGPU_INDEX_OFFSET = 1000


class ModelParser:
  """Handles parsing of model data into cereal objects"""

  @staticmethod
  def _parse_download_uri(download_uri_data) -> custom.ModelManagerSP.DownloadUri:
    download_uri = custom.ModelManagerSP.DownloadUri()
    download_uri.uri = download_uri_data.get("url")
    download_uri.sha256 = download_uri_data.get("sha256")
    return download_uri

  @staticmethod
  def _parse_chunk(chunk_data) -> custom.ModelManagerSP.Chunk:
    chunk = custom.ModelManagerSP.Chunk()
    chunk.fileName = chunk_data.get("file_name")
    chunk.sha256 = chunk_data.get("sha256")
    return chunk

  @staticmethod
  def _parse_artifact(artifact_data) -> custom.ModelManagerSP.Artifact:
    artifact = custom.ModelManagerSP.Artifact()
    artifact.fileName = artifact_data.get("file_name")
    artifact.downloadUri = ModelParser._parse_download_uri(artifact_data.get("download_uri", {}))

    if "chunks" in artifact_data:
      artifact.chunks = [ModelParser._parse_chunk(chunk_data) for chunk_data in artifact_data["chunks"]]

    return artifact

  @staticmethod
  def _parse_model(model_data) -> custom.ModelManagerSP.Model:
    model = custom.ModelManagerSP.Model()

    model.type = model_data.get("type")
    model.artifact = ModelParser._parse_artifact(model_data.get("artifact", {}))
    return model

  @staticmethod
  def _parse_overrides(overrides_data: dict[str, str]) -> list[custom.ModelManagerSP.Override]:
    overrides = []
    for key, value in overrides_data.items():
      override = custom.ModelManagerSP.Override()
      override.key = key
      override.value = value
      overrides.append(override)
    return overrides

  @staticmethod
  def _parse_bundle(bundle, *, index_offset: int = 0, platform: str | None = None) -> custom.ModelManagerSP.ModelBundle:
    model_bundle = custom.ModelManagerSP.ModelBundle()
    model_bundle.index = int(bundle["index"]) + index_offset
    model_bundle.internalName = bundle["short_name"]
    model_bundle.displayName = bundle["display_name"]
    model_bundle.models = [ModelParser._parse_model(model) for model in bundle.get("models",[])]
    model_bundle.status = 0
    model_bundle.generation = int(bundle["generation"])
    model_bundle.environment = bundle["environment"]
    model_bundle.runner = bundle.get("runner", custom.ModelManagerSP.Runner.snpe)
    model_bundle.is20hz = bundle.get("is_20hz", False)
    model_bundle.minimumSelectorVersion = int(bundle["minimum_selector_version"])
    overrides = dict(bundle.get("overrides", {}))
    if platform is not None:
      overrides["model_platform"] = platform
    model_bundle.overrides = ModelParser._parse_overrides(overrides)
    model_bundle.ref = bundle.get("ref")

    return model_bundle

  @staticmethod
  def parse_models(json_data: dict, *, index_offset: int = 0, platform: str | None = None) -> list[custom.ModelManagerSP.ModelBundle]:
    found_bundles = [ModelParser._parse_bundle(bundle, index_offset=index_offset, platform=platform)
                     for bundle in json_data.get("bundles", [])]
    return [bundle for bundle in found_bundles if is_bundle_version_compatible(bundle.to_dict())]


class ModelCache:
  """Handles caching of model data to avoid frequent remote fetches"""

  def __init__(self, params: Params, cache_timeout: int = int(3600 * 1e9), suffix: str = ""):
    self.params = params
    self.cache_timeout = cache_timeout
    self._LAST_SYNC_KEY = f"ModelManager_LastSyncTime{suffix}"
    self._CACHE_KEY = f"ModelManager_ModelsCache{suffix}"

  def _is_expired(self) -> bool:
    """Checks if the cache has expired"""
    current_time = int(time.monotonic() * 1e9)
    last_sync = self.params.get(self._LAST_SYNC_KEY) or 0
    return bool(last_sync == 0) or (current_time - last_sync) >= self.cache_timeout

  def get(self) -> tuple[dict, bool]:
    """
    Retrieves cached model data and expiration status atomically.
    Returns: Tuple of (cached_data, is_expired)
    If no cached data exists or on error, returns an empty dict
    """
    try:
      cached_data = self.params.get(self._CACHE_KEY)
      if not cached_data:
        cloudlog.warning("No cached model data available")
        return {}, True
      return cached_data, self._is_expired()
    except Exception as e:
      cloudlog.exception(f"Error retrieving cached model data: {str(e)}")
      return {}, True

  def set(self, data: dict) -> None:
    """Updates the cache with new model data"""
    self.params.put(self._CACHE_KEY, data, block=True)
    self.params.put(self._LAST_SYNC_KEY, int(time.monotonic() * 1e9), block=True)


class ModelFetcher:
  """Handles fetching and caching of model data from remote source"""
  MODEL_URL = "https://raw.githubusercontent.com/sunnypilot/sunnypilot-models/refs/heads/gh-pages/docs/driving_models_v21.json"
  MODEL_URL_USBGPU = "https://raw.githubusercontent.com/sunnypilot/sunnypilot-models/refs/heads/gh-pages/docs/driving_models_usbgpu_v22.json"

  def __init__(self, params: Params):
    self.params = params
    self.model_parser = ModelParser()
    self.model_cache = ModelCache(params)
    self.usbgpu_model_cache = ModelCache(params, suffix="_USBGPU")
    self.model_url = self.MODEL_URL
    self.params.put("ModelManager_ActiveJson", f"{self.MODEL_URL};{self.MODEL_URL_USBGPU}", block=True)

  def _fetch_and_cache_models(self, model_url: str, model_cache: ModelCache, *,
                              index_offset: int, platform: str) -> list[custom.ModelManagerSP.ModelBundle] | None:
    """Fetches fresh model data from remote and updates cache.
    Returns None on transport errors. Raises on 404 and other fatal HTTP errors.
    """
    try:
      response = requests.get(model_url, timeout=10)

      # Explicitly handle 404 differently
      if response.status_code == 404:
        cloudlog.error(f"Models URL returned 404 Not Found: {self.model_url}")
        raise HTTPError(f"404 Not Found: {self.model_url}", response=response)

      # Raise for any other 4xx/5xx
      response.raise_for_status()

      json_data = response.json()
      model_cache.set(json_data)
      cloudlog.debug("Successfully updated models cache")
      return self.model_parser.parse_models(json_data, index_offset=index_offset, platform=platform)

    except ConnectionError as e:
      cloudlog.warning(f"DNS/connection error while fetching models: {e}")
    except SSLError as e:
      cloudlog.warning(f"SSL error while fetching models: {e}")
    except RequestException as e:
      cloudlog.warning(f"Request transport error while fetching models: {e}")
    except Exception as e:
      cloudlog.exception(f"Unexpected error fetching models: {e}")

    return None

  def _get_catalog(self, model_url: str, model_cache: ModelCache, *,
                   index_offset: int, platform: str) -> list[custom.ModelManagerSP.ModelBundle]:
    cached_data, is_expired = model_cache.get()

    if cached_data and not is_expired:
      cloudlog.debug("Using valid cached models data")
      return self.model_parser.parse_models(cached_data, index_offset=index_offset, platform=platform)

    fetched_bundles = self._fetch_and_cache_models(model_url, model_cache, index_offset=index_offset, platform=platform)
    if fetched_bundles is not None:
      return fetched_bundles

    if not cached_data:
      cloudlog.warning("Failed to fetch fresh data and no cache available")

    cloudlog.warning("Failed to fetch fresh data. Using expired cache as fallback")
    return self.model_parser.parse_models(cached_data, index_offset=index_offset, platform=platform)

  def get_available_bundles(self) -> list[custom.ModelManagerSP.ModelBundle]:
    """Return QCOM and USBGPU catalogs together with stable, non-overlapping indices."""
    small = self._get_catalog(self.MODEL_URL, self.model_cache, index_offset=0, platform="qcom")
    big = self._get_catalog(self.MODEL_URL_USBGPU, self.usbgpu_model_cache,
                            index_offset=USBGPU_INDEX_OFFSET, platform="usbgpu")
    return [*small, *big]

if __name__ == "__main__":
  params = Params()
  model_fetcher = ModelFetcher(params)
  bundles = model_fetcher.get_available_bundles()
  for bundle in bundles:
    for model in bundle.models:
      model_overrides = {override.key: override.value for override in bundle.overrides}
      print(f"Bundle: {bundle.internalName}, Type: {model.type}, Status: {bundle.status}, Overrides: {model_overrides}")
      print(f"Artifact: {model.artifact.fileName}, Download URI: {model.artifact.downloadUri.uri}")
      if model.artifact.chunks:
        print(f"Contains {len(model.artifact.chunks)} chunks.")
