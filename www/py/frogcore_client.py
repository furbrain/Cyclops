"""Client for uploading Cyclops surveys to FrogCore (drop into the CM5 Flask app)."""
import pathlib

import requests


class FrogCoreClient:
    def __init__(self, base_url: str, username: str, password: str):
        self.base = base_url.rstrip("/")
        self.http = requests.Session()
        r = self.http.post(f"{self.base}/api/login", auth=(username, password), timeout=30)
        r.raise_for_status()
        self.http.headers["Authorization"] = f"Bearer {r.json()['token']}"

    def create_survey(self, name: str) -> str:
        r = self.http.post(f"{self.base}/api/surveys", json={"name": name}, timeout=30)
        r.raise_for_status()
        return r.json()["id"]

    def upload(self, survey_id: str, path: pathlib.Path) -> None:
        with path.open("rb") as fh:  # requests sets Content-Length from the file size
            r = self.http.put(f"{self.base}/api/surveys/{survey_id}/files/{path.name}",
                              data=fh, timeout=600)
        r.raise_for_status()

    def build(self, survey_id: str) -> None:
        self.http.post(f"{self.base}/api/surveys/{survey_id}/build", timeout=30).raise_for_status()
