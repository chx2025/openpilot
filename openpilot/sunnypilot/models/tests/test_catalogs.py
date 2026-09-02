from openpilot.sunnypilot.models.fetcher import ModelFetcher, ModelParser
from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION, is_bundle_version_compatible


def bundle_json(*, index=0, ref="model-ref", selector=REQUIRED_JSON_VERSION, is_big=False):
  return {
    "index": index,
    "short_name": "TEST",
    "display_name": "Test Model",
    "generation": 12,
    "environment": "development",
    "runner": "tinygrad",
    "is_20hz": True,
    "minimum_selector_version": selector,
    "ref": ref,
    "is_big": is_big,
    "overrides": {},
    "models": [],
  }


def test_catalogs_use_official_qcom_and_chestnut_sources_without_rewriting_metadata():
  assert set(ModelFetcher.MODEL_SOURCES) == {"qcom", "chestnut"}
  assert ModelFetcher.MODEL_SOURCES["qcom"][0].endswith("driving_models_v22.json")
  assert ModelFetcher.MODEL_SOURCES["chestnut"][0].endswith("driving_models_chestnut_v23.json")

  bundle = ModelParser.parse_models({"bundles": [bundle_json(index=4, is_big=True)]})[0]

  assert bundle.index == 4
  assert {override.key for override in bundle.overrides} == set()


def test_only_current_selector_version_is_accepted():
  assert is_bundle_version_compatible({"minimumSelectorVersion": REQUIRED_JSON_VERSION})
  assert not is_bundle_version_compatible({"minimumSelectorVersion": REQUIRED_JSON_VERSION - 1})
