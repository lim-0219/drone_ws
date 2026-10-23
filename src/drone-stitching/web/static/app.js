"use strict";

const EXPECTED_IMAGE_COUNT = 9;
const POLL_INTERVAL_MS = 1_100;
const POLL_TIMEOUT_MS = 10 * 60 * 1_000;
const MAX_NETWORK_RETRIES = 4;
const SESSION_POLL_INTERVAL_MS = 1_100;

const elements = {
  captureCount: document.querySelector("#capture-count"),
  captureMessage: document.querySelector("#capture-message"),
  captureProgress: document.querySelector("#capture-progress"),
  captureProgressBar: document.querySelector("#capture-progress-bar"),
  clearButton: document.querySelector("#clear-button"),
  completedState: document.querySelector("#completed-state"),
  dropZone: document.querySelector("#drop-zone"),
  emptyDescription: document.querySelector("#empty-description"),
  emptyState: document.querySelector("#empty-state"),
  failedState: document.querySelector("#failed-state"),
  failureMessage: document.querySelector("#failure-message"),
  formMessage: document.querySelector("#form-message"),
  frameGrid: document.querySelector("#frame-grid"),
  frameSummary: document.querySelector("#frame-summary"),
  imageInput: document.querySelector("#image-input"),
  jobReference: document.querySelector("#job-reference"),
  mapImage: document.querySelector("#map-image"),
  mapOpenLink: document.querySelector("#map-open-link"),
  mapStage: document.querySelector("#map-stage"),
  overallStatus: document.querySelector("#overall-status"),
  retryButton: document.querySelector("#retry-button"),
  scanPhase: document.querySelector("#scan-phase"),
  scanningState: document.querySelector("#scanning-state"),
  scanSteps: [...document.querySelectorAll("[data-scan-step]")],
};

const naturalNameOrder = new Intl.Collator("ko-KR", {
  numeric: true,
  sensitivity: "base",
});

const appState = {
  files: [],
  previewUrls: [],
  jobId: null,
  busy: false,
  mode: "idle",
  runId: 0,
  requestController: null,
  autoStartTimer: null,
  externalSessionId: null,
  externalSessionStatus: null,
  externalMapUrl: null,
};

function initialise() {
  renderFileCollection();

  elements.dropZone.addEventListener("click", openFilePicker);
  elements.dropZone.addEventListener("keydown", (event) => {
    if (event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      openFilePicker();
    }
  });

  elements.imageInput.addEventListener("change", () => {
    acceptFiles([...elements.imageInput.files]);
    elements.imageInput.value = "";
  });

  ["dragenter", "dragover"].forEach((eventName) => {
    elements.dropZone.addEventListener(eventName, (event) => {
      event.preventDefault();
      event.stopPropagation();
      if (!appState.busy) {
        elements.dropZone.classList.add("dragging");
      }
    });
  });

  ["dragleave", "drop"].forEach((eventName) => {
    elements.dropZone.addEventListener(eventName, (event) => {
      event.preventDefault();
      event.stopPropagation();
      elements.dropZone.classList.remove("dragging");
    });
  });

  elements.dropZone.addEventListener("drop", (event) => {
    if (!appState.busy) {
      acceptFiles([...event.dataTransfer.files]);
    }
  });

  elements.clearButton.addEventListener("click", () => resetApplication());
  elements.retryButton.addEventListener("click", () => {
    resetApplication();
    elements.imageInput.click();
  });

  window.addEventListener("beforeunload", cancelActiveWork);
  window.setInterval(pollExternalSession, SESSION_POLL_INTERVAL_MS);
  pollExternalSession();
}

// --- drone sessions (frames arrive through POST /api/sessions/<id>/frames) ---
// While the operator has not picked files by hand, mirror the newest session
// the ground station is feeding: n/9 received, Map Scanning, then the map.
async function pollExternalSession() {
  if (appState.busy || appState.files.length > 0) {
    return;
  }
  let session;
  try {
    const response = await fetch("/api/sessions/latest", {
      cache: "no-store",
      headers: { Accept: "application/json" },
    });
    if (response.status === 404) {
      return;
    }
    if (!response.ok) {
      return;
    }
    session = await response.json();
  } catch {
    return;
  }
  const signature = `${session.id}:${session.status}:${session.received}:${session.updated_at}`;
  if (signature === appState.externalSessionStatus) {
    return;
  }
  appState.externalSessionStatus = signature;
  appState.externalSessionId = session.id;
  renderExternalSession(session);
}

function renderExternalSession(session) {
  const count = Number(session.received) || 0;
  const label = `드론 세션 ${abbreviateJobId(String(session.id))}`;
  elements.captureCount.textContent = String(count);
  elements.captureProgress.setAttribute("aria-valuenow", String(count));
  elements.captureProgressBar.style.width = `${(count / EXPECTED_IMAGE_COUNT) * 100}%`;
  elements.frameSummary.textContent = `${count}장 수신 · 드론에서 직접 전송`;

  const received = new Set((session.frames || []).map((frame) => Number(frame.sequence)));
  const cells = (session.capture_order || []).map(String);
  elements.frameGrid.replaceChildren();
  for (let index = 0; index < EXPECTED_IMAGE_COUNT; index += 1) {
    const slot = document.createElement("li");
    const sequence = index + 1;
    slot.className = `frame-slot${received.has(sequence) ? " filled" : ""}`;
    const number = document.createElement("span");
    number.className = "frame-number";
    number.textContent = String(sequence).padStart(2, "0");
    slot.append(number);
    const mark = document.createElement("span");
    mark.className = "frame-empty-mark";
    mark.textContent = received.has(sequence) ? `셀 ${cells[index] ?? ""}` : "대기";
    slot.append(mark);
    elements.frameGrid.append(slot);
  }

  const status = String(session.status || "collecting").toLowerCase();
  if (status === "collecting") {
    setStatusChip(`드론 수신 중 ${count}/${EXPECTED_IMAGE_COUNT}`, "collecting");
    elements.captureMessage.textContent = `드론이 보낸 ${count}장을 받았습니다. 9장이 모이면 자동으로 지도를 만듭니다.`;
    elements.emptyDescription.textContent = `드론 세션 수신 중 (${count}/${EXPECTED_IMAGE_COUNT}).`;
    elements.jobReference.textContent = label;
    showMapPanel("empty");
    return;
  }
  if (status === "queued" || status === "scanning") {
    showScanningState(status === "queued" ? "queue" : "stitch",
      status === "queued" ? "9장을 받았습니다. 스티칭 순서를 준비하고 있습니다." : "특징점을 분석하고 2D 지도를 합성하고 있습니다.");
    setStatusChip("맵 스캔 중", "running");
    elements.captureMessage.textContent = "9장 수신 완료 · Map Scanning";
    elements.jobReference.textContent = `${label} · 스캔 중`;
    return;
  }
  if (status === "completed") {
    const mapUrl = session.map_url;
    if (!mapUrl || mapUrl === appState.externalMapUrl) {
      return;
    }
    appState.externalMapUrl = mapUrl;
    const versionToken = encodeURIComponent(session.updated_at || Date.now());
    elements.mapImage.onload = () => {
      elements.mapOpenLink.href = mapUrl;
      showMapPanel("completed");
      setStatusChip("완료", "success");
      elements.captureMessage.textContent = "드론 세션 9장의 자동 스티칭이 완료되었습니다.";
      elements.frameSummary.textContent = "9장 처리 완료 · 2D 지도 생성 성공";
      elements.jobReference.textContent = `${label} · 완료`;
    };
    elements.mapImage.onerror = () => {
      elements.failureMessage.textContent = "생성된 지도 이미지를 불러올 수 없습니다.";
      showMapPanel("failed");
    };
    elements.mapImage.src = `${mapUrl}?v=${versionToken}`;
    return;
  }
  if (status === "failed") {
    elements.failureMessage.textContent = humaniseError(session.error || "스티칭 처리 중 오류가 발생했습니다.");
    showMapPanel("failed");
    setStatusChip("실패", "error");
    elements.captureMessage.textContent = "드론 세션 지도 생성 실패 · 서버 진단을 확인해 주세요.";
    elements.jobReference.textContent = `${label} · 실패`;
  }
}

function openFilePicker() {
  if (!appState.busy) {
    elements.imageInput.click();
  }
}

function acceptFiles(incomingFiles) {
  if (appState.busy || incomingFiles.length === 0) {
    return;
  }
  appState.externalSessionStatus = null;
  appState.externalMapUrl = null;

  if (appState.mode === "completed" || appState.mode === "failed") {
    resetApplication();
  }

  hideFormMessage();

  const unsupported = incomingFiles.filter((file) => !isSupportedImage(file));
  if (unsupported.length > 0) {
    showFormMessage(
      `이미지 파일만 선택할 수 있습니다: ${unsupported
        .slice(0, 2)
        .map((file) => file.name)
        .join(", ")}${unsupported.length > 2 ? " 외" : ""}`,
    );
    return;
  }

  const existingKeys = new Set(appState.files.map(fileIdentity));
  const uniqueIncoming = incomingFiles.filter((file) => {
    const key = fileIdentity(file);
    if (existingKeys.has(key)) {
      return false;
    }
    existingKeys.add(key);
    return true;
  });

  const nextFiles = [...appState.files, ...uniqueIncoming];
  if (nextFiles.length > EXPECTED_IMAGE_COUNT) {
    showFormMessage(
      `이미지는 정확히 9장이어야 합니다. 현재 ${appState.files.length}장에 ${uniqueIncoming.length}장을 더하면 ${nextFiles.length}장입니다.`,
    );
    return;
  }

  if (uniqueIncoming.length === 0) {
    showFormMessage("이미 선택된 이미지입니다. 다른 파일을 선택해 주세요.");
    return;
  }

  appState.files = nextFiles.sort(compareFilesNaturally);
  appState.mode = appState.files.length === EXPECTED_IMAGE_COUNT ? "ready" : "collecting";
  renderFileCollection();

  if (appState.files.length === EXPECTED_IMAGE_COUNT) {
    appState.autoStartTimer = window.setTimeout(startStitchingJob, 450);
  }
}

function isSupportedImage(file) {
  if (file.type.startsWith("image/")) {
    return true;
  }
  return /\.(?:jpe?g|png|webp|bmp|tiff?)$/i.test(file.name);
}

function fileIdentity(file) {
  return `${file.name}\u0000${file.size}\u0000${file.lastModified}`;
}

function compareFilesNaturally(left, right) {
  return (
    naturalNameOrder.compare(left.name, right.name) ||
    left.lastModified - right.lastModified ||
    left.size - right.size
  );
}

function renderFileCollection() {
  const count = appState.files.length;
  elements.captureCount.textContent = String(count);
  elements.captureProgress.setAttribute("aria-valuenow", String(count));
  elements.captureProgressBar.style.width = `${(count / EXPECTED_IMAGE_COUNT) * 100}%`;
  elements.clearButton.hidden = count === 0 || appState.busy;

  revokePreviewUrls();
  elements.frameGrid.replaceChildren();

  for (let index = 0; index < EXPECTED_IMAGE_COUNT; index += 1) {
    const file = appState.files[index];
    const slot = document.createElement("li");
    slot.className = `frame-slot${file ? " filled" : ""}`;

    const number = document.createElement("span");
    number.className = "frame-number";
    number.textContent = String(index + 1).padStart(2, "0");
    slot.append(number);

    if (file) {
      const fallback = document.createElement("span");
      fallback.className = "frame-empty-mark";
      fallback.textContent = "IMG";
      slot.append(fallback);

      const previewUrl = URL.createObjectURL(file);
      appState.previewUrls.push(previewUrl);

      const image = document.createElement("img");
      image.src = previewUrl;
      image.alt = `${index + 1}번 이미지 미리보기: ${file.name}`;
      image.addEventListener("error", () => image.remove(), { once: true });
      slot.append(image);

      const fileName = document.createElement("span");
      fileName.className = "frame-name";
      fileName.textContent = file.name;
      slot.title = `${index + 1}번 · ${file.name}`;
      slot.append(fileName);
    } else {
      const emptyMark = document.createElement("span");
      emptyMark.className = "frame-empty-mark";
      emptyMark.textContent = "대기";
      slot.append(emptyMark);
      slot.setAttribute("aria-label", `${index + 1}번 이미지 대기 중`);
    }

    elements.frameGrid.append(slot);
  }

  updateCollectionCopy(count);
}

function updateCollectionCopy(count) {
  if (count === 0) {
    setStatusChip("대기", "neutral");
    elements.captureMessage.textContent =
      "지도 생성을 시작하려면 이미지 9장을 선택하세요.";
    elements.emptyDescription.textContent = "9장의 드론 이미지를 기다리고 있습니다.";
    elements.frameSummary.textContent = "아직 선택된 이미지가 없습니다.";
    return;
  }

  if (count < EXPECTED_IMAGE_COUNT) {
    const remaining = EXPECTED_IMAGE_COUNT - count;
    setStatusChip("수신 중", "collecting");
    elements.captureMessage.textContent = `${count}장 준비됨 · ${remaining}장을 더 선택해 주세요.`;
    elements.emptyDescription.textContent = `${count}장의 이미지가 준비됐습니다. ${remaining}장을 더 기다리고 있습니다.`;
    elements.frameSummary.textContent = `${count}장 준비 · 파일명 순으로 자동 정렬`;
    return;
  }

  if (!appState.busy && appState.mode === "ready") {
    setStatusChip("자동 시작", "running");
    elements.captureMessage.textContent =
      "9장 준비 완료 · 잠시 후 자동 스티칭을 시작합니다.";
  }
  elements.frameSummary.textContent = "9장 준비 완료 · 파일명 순으로 자동 정렬";
}

async function startStitchingJob() {
  appState.autoStartTimer = null;
  if (appState.busy || appState.files.length !== EXPECTED_IMAGE_COUNT) {
    return;
  }

  appState.busy = true;
  appState.mode = "uploading";
  appState.runId += 1;
  const currentRunId = appState.runId;
  appState.requestController = new AbortController();

  setInteractionLocked(true);
  hideFormMessage();
  showScanningState("upload", "이미지 9장을 서버로 전송하고 있습니다.");
  setStatusChip("전송 중", "running");
  elements.captureMessage.textContent = "9장 수신 완료 · 서버 전송 중";
  elements.jobReference.textContent = "새 작업 생성 중";

  const requestBody = new FormData();
  appState.files.forEach((file) => requestBody.append("images", file, file.name));

  try {
    const response = await fetch("/api/jobs", {
      method: "POST",
      body: requestBody,
      signal: appState.requestController.signal,
    });

    const responseData = await readResponseData(response);
    if (!response.ok) {
      throw new Error(extractApiError(responseData, `업로드 실패 (${response.status})`));
    }

    const jobId = responseData.job_id ?? responseData.id;
    if (!jobId) {
      throw new Error("서버 응답에 작업 ID가 없습니다.");
    }

    if (currentRunId !== appState.runId) {
      return;
    }

    appState.jobId = String(jobId);
    elements.jobReference.textContent = `작업 ${abbreviateJobId(appState.jobId)}`;

    const finished = await applyJobStatus(responseData, currentRunId);
    if (!finished) {
      await pollJobUntilFinished(currentRunId);
    }
  } catch (error) {
    if (error.name !== "AbortError" && currentRunId === appState.runId) {
      failJob(error.message || "지도 생성 요청 중 알 수 없는 오류가 발생했습니다.");
    }
  }
}

async function pollJobUntilFinished(currentRunId) {
  const deadline = Date.now() + POLL_TIMEOUT_MS;
  let networkFailures = 0;

  while (
    appState.busy &&
    currentRunId === appState.runId &&
    Date.now() < deadline
  ) {
    await wait(POLL_INTERVAL_MS);

    if (!appState.busy || currentRunId !== appState.runId) {
      return;
    }

    try {
      const response = await fetch(`/api/jobs/${encodeURIComponent(appState.jobId)}`, {
        cache: "no-store",
        headers: { Accept: "application/json" },
        signal: appState.requestController.signal,
      });
      const responseData = await readResponseData(response);

      if (!response.ok) {
        throw new Error(
          extractApiError(responseData, `작업 상태 확인 실패 (${response.status})`),
        );
      }

      networkFailures = 0;
      const finished = await applyJobStatus(responseData, currentRunId);
      if (finished) {
        return;
      }
    } catch (error) {
      if (error.name === "AbortError" || currentRunId !== appState.runId) {
        return;
      }

      networkFailures += 1;
      if (networkFailures > MAX_NETWORK_RETRIES) {
        failJob(error.message || "서버와 연결할 수 없습니다.");
        return;
      }

      showScanningState(
        "queue",
        `서버 연결을 다시 확인하고 있습니다 (${networkFailures}/${MAX_NETWORK_RETRIES}).`,
      );
    }
  }

  if (appState.busy && currentRunId === appState.runId) {
    failJob("지도 생성 대기 시간이 10분을 초과했습니다. 서버 상태를 확인해 주세요.");
  }
}

async function applyJobStatus(job, currentRunId) {
  const status = String(job.status || "").toLowerCase();

  if (status === "queued") {
    appState.mode = "queued";
    showScanningState("queue", "이미지를 확인하고 스티칭 순서를 준비하고 있습니다.");
    setStatusChip("처리 대기", "running");
    elements.captureMessage.textContent = "9장 수신 완료 · 자동 처리 대기 중";
    return false;
  }

  if (status === "scanning") {
    appState.mode = "scanning";
    showScanningState("stitch", "특징점을 분석하고 2D 지도를 합성하고 있습니다.");
    setStatusChip("맵 스캔 중", "running");
    elements.captureMessage.textContent = "9장 수신 완료 · Map Scanning";
    return false;
  }

  if (status === "completed") {
    appState.mode = "loading-map";
    showScanningState("stitch", "합성된 지도를 불러오고 있습니다.");
    const mapUrl = job.map_url || `/api/jobs/${encodeURIComponent(appState.jobId)}/map`;
    await showCompletedMap(mapUrl, currentRunId, job.updated_at);
    return true;
  }

  if (status === "failed") {
    failJob(job.error || "스티칭 처리 중 오류가 발생했습니다.");
    return true;
  }

  if (!status) {
    throw new Error("서버가 작업 상태를 반환하지 않았습니다.");
  }

  throw new Error(`지원하지 않는 작업 상태입니다: ${status}`);
}

function showCompletedMap(mapUrl, currentRunId, version) {
  return new Promise((resolve) => {
    const separator = mapUrl.includes("?") ? "&" : "?";
    const versionToken = encodeURIComponent(version || Date.now());
    const resolvedUrl = `${mapUrl}${separator}v=${versionToken}`;

    elements.mapImage.onload = () => {
      if (currentRunId !== appState.runId) {
        resolve();
        return;
      }

      appState.busy = false;
      appState.mode = "completed";
      appState.requestController = null;
      elements.mapOpenLink.href = mapUrl;
      showMapPanel("completed");
      setInteractionLocked(false);
      setStatusChip("완료", "success");
      elements.captureMessage.textContent = "9장 자동 스티칭이 완료되었습니다.";
      elements.frameSummary.textContent = "9장 처리 완료 · 2D 지도 생성 성공";
      elements.jobReference.textContent = `작업 ${abbreviateJobId(appState.jobId)} · 완료`;
      resolve();
    };

    elements.mapImage.onerror = () => {
      if (currentRunId === appState.runId) {
        failJob("생성된 지도 이미지를 불러올 수 없습니다.");
      }
      resolve();
    };

    elements.mapImage.src = resolvedUrl;
  });
}

function failJob(message) {
  appState.busy = false;
  appState.mode = "failed";
  appState.requestController = null;
  elements.failureMessage.textContent = humaniseError(message);
  showMapPanel("failed");
  setInteractionLocked(false);
  setStatusChip("실패", "error");
  elements.captureMessage.textContent = "지도 생성 실패 · 이미지를 다시 선택해 주세요.";
  elements.frameSummary.textContent = `${appState.files.length}장 처리 실패 · 입력 이미지 확인 필요`;
  elements.jobReference.textContent = appState.jobId
    ? `작업 ${abbreviateJobId(appState.jobId)} · 실패`
    : "작업 생성 실패";
}

function showScanningState(step, message) {
  showMapPanel("scanning");
  elements.scanPhase.textContent = message;

  const stepOrder = ["upload", "queue", "stitch"];
  const activeIndex = stepOrder.indexOf(step);
  elements.scanSteps.forEach((item, index) => {
    item.classList.toggle("active", index === activeIndex);
    item.classList.toggle("done", index < activeIndex);
  });
}

function showMapPanel(panel) {
  elements.emptyState.hidden = panel !== "empty";
  elements.scanningState.hidden = panel !== "scanning";
  elements.completedState.hidden = panel !== "completed";
  elements.failedState.hidden = panel !== "failed";
  elements.mapStage.setAttribute(
    "aria-busy",
    panel === "scanning" ? "true" : "false",
  );
}

function setInteractionLocked(locked) {
  elements.dropZone.classList.toggle("disabled", locked);
  elements.dropZone.setAttribute("aria-disabled", String(locked));
  elements.imageInput.disabled = locked;
  elements.clearButton.hidden = locked || appState.files.length === 0;
}

function setStatusChip(label, tone) {
  elements.overallStatus.textContent = label;
  elements.overallStatus.className = `status-chip ${tone}`;
}

function showFormMessage(message) {
  elements.formMessage.textContent = message;
  elements.formMessage.hidden = false;
}

function hideFormMessage() {
  elements.formMessage.hidden = true;
  elements.formMessage.textContent = "";
}

function resetApplication() {
  cancelActiveWork();
  appState.runId += 1;
  appState.files = [];
  appState.jobId = null;
  appState.busy = false;
  appState.mode = "idle";
  appState.requestController = null;
  elements.imageInput.disabled = false;
  elements.imageInput.value = "";
  elements.dropZone.classList.remove("disabled", "dragging");
  elements.dropZone.setAttribute("aria-disabled", "false");
  elements.mapImage.onload = null;
  elements.mapImage.onerror = null;
  elements.mapImage.removeAttribute("src");
  elements.mapOpenLink.href = "#";
  elements.jobReference.textContent = "새 작업 대기 중";
  hideFormMessage();
  showMapPanel("empty");
  renderFileCollection();
}

function cancelActiveWork() {
  if (appState.autoStartTimer !== null) {
    window.clearTimeout(appState.autoStartTimer);
    appState.autoStartTimer = null;
  }
  if (appState.requestController) {
    appState.requestController.abort();
  }
}

function revokePreviewUrls() {
  appState.previewUrls.forEach((url) => URL.revokeObjectURL(url));
  appState.previewUrls = [];
}

function abbreviateJobId(jobId) {
  return jobId.length > 12 ? `${jobId.slice(0, 8)}…` : jobId;
}

function wait(milliseconds) {
  return new Promise((resolve) => window.setTimeout(resolve, milliseconds));
}

async function readResponseData(response) {
  const contentType = response.headers.get("content-type") || "";
  if (contentType.includes("application/json")) {
    try {
      return await response.json();
    } catch {
      return {};
    }
  }

  try {
    return { detail: await response.text() };
  } catch {
    return {};
  }
}

function extractApiError(data, fallback) {
  if (!data) {
    return fallback;
  }

  if (typeof data.error === "string" && data.error.trim()) {
    return data.error;
  }
  if (typeof data.detail === "string" && data.detail.trim()) {
    return data.detail;
  }
  if (Array.isArray(data.detail)) {
    const messages = data.detail
      .map((item) => (typeof item === "string" ? item : item?.msg))
      .filter(Boolean);
    if (messages.length > 0) {
      return messages.join(" · ");
    }
  }
  if (typeof data.message === "string" && data.message.trim()) {
    return data.message;
  }
  return fallback;
}

function humaniseError(message) {
  if (typeof message !== "string" || !message.trim()) {
    return "알 수 없는 오류가 발생했습니다. 이미지를 확인한 뒤 다시 시도해 주세요.";
  }
  if (/failed to fetch|networkerror|load failed/i.test(message)) {
    return "로컬 서버에 연결할 수 없습니다. 서버가 실행 중인지 확인해 주세요.";
  }
  return message.trim();
}

initialise();
