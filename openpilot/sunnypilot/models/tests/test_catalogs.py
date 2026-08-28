from openpilot.sunnypilot.models.fetcher import ModelFetcher, ModelParser
from openpilot.sunnypilot.models.helpers import is_bundle_version_compatible


def bundle_json(*, index=0, ref="model-ref", selector=18, is_big=False):
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
  assert ModelFetcher.MODEL_SOURCES["chestnut"][0].endswith("driving_models_chestnut_v22.json")

  bundle = ModelParser.parse_models({"bundles": [bundle_json(index=4, is_big=True)]})[0]

  assert bundle.index == 4
  assert {override.key for override in bundle.overrides} == set()


def test_only_current_selector_version_is_accepted():
  assert is_bundle_version_compatible({"minimumSelectorVersion": 18})
  assert not is_bundle_version_compatible({"minimumSelectorVersion": 17})
