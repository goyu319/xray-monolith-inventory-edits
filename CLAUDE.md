# CLAUDE.md — GAMMA カスタムエンジン & MOD 環境

S.T.A.L.K.E.R. Anomaly + GAMMA 環境の作業メモ（エンジン開発部分）。このファイルには、これまでの作業の経緯と決定事項、次に作業するときの注意点をまとめる。

## 環境

- S.T.A.L.K.E.R. Anomaly + GAMMA（MO2 プロファイルは Squared Away）
- **使う exe**：**AnomalyDX11AVX.exe**
  - 自前マージのエンジンでは、AnomalyDX12.exe は使わない（Squared Away の assert で落ちたことがある）
- PC 環境や自作 MOD のメモは、公開しないためにリポジトリ外の個人用メモ（CLAUDE.local.md）に分けてある。必要ならユーザーに貼ってもらう

## エンジン（自前マージ版）

### リポジトリ

- **フォーク**：https://github.com/goyu319/xray-monolith-inventory-edits
- **ブランチ**：`sa-fg-v04-bc031`
- **リモート**：

  | 名前 | リポジトリ |
  |---|---|
  | fomok | Fomok/xray-monolith-inventory-edits |
  | bodycam | asuparabekon/xray-monolith-bodycam |
  | origin | smoke-0n-the-water/stalker-dlss-fg-bodycam-pip |
  | fork | goyu319 |

- **注意**：クラウドの作業セッションで直接クローンした場合、`origin` は goyu319 のフォークになる
- **Actions の手動実行**：`amp-bodycam-build.yml` は workflow_dispatch に対応。push で自動ビルドされるのは `sa-fg-v04` と `sa-fg-v04-bc031` などだけなので、別ブランチは `gh workflow run amp-bodycam-build.yml --ref <ブランチ>` で動かす
- **ビルド**：GitHub Actions の `.github/workflows/amp-bodycam-build.yml`
  - 成果物の名前は `Squared-Away-DLSS-FG-v04-Bodycam031-DX11-AVX`
  - 作業環境からは成果物をダウンロードできない。ユーザーに Actions のページから取ってもらい、Anomaly の `bin` フォルダに上書きしてもらう

### 構成（ベースの上に順番に重ねたもの）

1. **ベース**：Fomok の「Squared Away + Bodycam + DLSS + FG」（コミット `fa1b97c3`）
2. **smoke-0n-the-water v0.4**：TBB の導入、テクスチャ常駐管理（Texture Residency）、tex_wait_log など
3. **Immersive Bodycam 0.3.1** のマージ（タグ `2026.10.25-bodycam-mt`、コミット `944d3dd84`）
4. **Vektor のカクつき修正**（`42641cbad`）
5. **自前修正：武器が真っ黒になるバグ**（`470591042`）
   - 場所：`dx10ResourceManager_Resources.cpp` の QueueTextureSwap
   - 修正内容：テクスチャは staging にせず読み込む。SRV を作れなかった差し替えは捨てる
6. **v0.4.1 の VRAM 自動調整**（`fabe7077c`）
7. **自前修正：屋内銃声のバグ**（`c357abf32`）。下の「屋内銃声バグ」を参照
   - Actions の実行 41194604813 でビルド成功、ゲーム内で直ったことを確認済み
8. **自前修正：デバイス所持で FG が止まるバグ**（`d456ce7`、ブランチ `fg-device-fix`）。下の「デバイス所持で FG が止まる件」を参照
   - Actions の実行 41274692429 でビルド中。**ゲーム内での確認はまだ**。確認できたら `sa-fg-v04-bc031` にマージする

### 屋内銃声バグ（2026-10-31 に解決）

- **症状**：Spatial Audio Rework（SAR）の屋内用銃声（`*_indoor`）に切り替わらず、建物の中の銃声がおかしくなる
- **原因**：`src/xrGame/HudSound.cpp` の `HUD_SOUND_COLLECTION_LAYERED::PlaySound`
  - `alias_to_play` が、`{ }` の中だけで有効な文字列 `xr_string new_alias` を指したままになっていた（ダングリングポインタ）
  - このバグは元のコードに前からあった。v0.4 でメモリ管理が tbbmalloc（`PURE_ALLOC_TBBMALLOC`）に変わり、捨てたメモリがすぐ再利用されるようになったので表に出た
- **修正**：`xr_string new_alias;` を関数の最初に移し、`{ }` の中では代入だけにする
  - 中で `xr_string new_alias = ...` と宣言し直すと、同じ名前の別の変数ができて、バグは直らない
- **上流への反映**
  - v0.4 の作者に報告し、修正が取り込まれた
  - 公式の v0.4.1 リリースのファイルは、2026-10-31 21:00 頃（日本時間）に同じタグのまま差し替えられた
  - その AnomalyDX12.exe（PE タイムスタンプ 11:59 UTC）を逆アセンブルし、修正が入っていることを確認済み
  - アップデート版の zip に入っているのは DX12 の exe だけ。フルパックの DX11 exe は未確認
- **パフォーマンスへの影響**：なし

### デバイス所持で FG が止まる件（2026-11-01）

- **症状**：ガイガーや探知機を手に持つと、FG オンで約 150FPS → 約 100FPS（FG オフと同じ）に落ちる。画面の有無に関係なく全デバイスで起きる。Fomok の DLSS 版の頃から発生
- **原因**：FG が意図的に止められていた。描画負荷ではない
  - ログに `[Personal FG] UI content flags=0x15` → `Suspended for embedded item/camera UI.` → `[DLSS FG] Interpolation suspended.` が出る
  - `src/Layers/xrRenderPC_R4/Dlss/PersonalFrameGeneration.cpp` の `Eligible()` が、flags の `0x1`（手持ちアイテムの 3D UI）で FG を止めていた
  - `0x1` は `CCustomDevice::has_item_3d_ui_content()`（`m_ui && IsWorking()`）で立つので、画面のないガイガーも対象になる
  - この判定は Fomok のコミット `a2bc697`（2026-10-18）由来
- **修正**：
  - `Eligible()` の停止条件を `(1u | 2u | 8u)` → `(2u | 8u)` にした
  - `ProtectEmbeddedUi()` の対象を `4u` → `(1u | 4u)` にした。デバイス画面の UI を、HUD attachment と同じアップスケール後のネイティブ描画パス（HUDless のコピーより後）に回すため
  - ユーザー環境では通常時の flags が `0x14`（`4u` が常時立っている）なので、デバイス UI は以前からこのパスで描かれていた
- **確認ポイント**：ログに `Suspended for embedded item/camera UI` が出ないこと。Veles などの画面付き探知機で、画面の縁にちらつきやゴーストが出ないこと
- 未対応：item attachment（`0x2`）と camera UI（`0x8`）は、まだ FG を止める

### 反響（EFX）について

- 反響を処理するエンジン側のコード（xrSound）は、Fomok のベースや公式 v0.4.1 と同じ
- **反響が起動しているかの確認**：ログに `SOUND: OpenAL: EFX supported` が出ていれば機能は起動している
- **EFX はゲーム起動時にしか準備されない**
  - SAR は、ゲームに入ってから `snd_efx 1` と `snd_efx_reverb_overwrite 1` を設定する
  - そのため、`snd_efx` がオフの状態で起動していた場合は、一度再起動するまで反響が効かない
- **SAR の MCM で ray debug をオンにしているあいだは、反響を一切かけない**（`sar_main.script` の 344 行目）。反響エディターがオンのときも同じ
- **切り分け方**：コンソールで `snd_efx_reverb_overwrite_gain` の値を屋内と屋外で比べる。変わらなければ SAR が反響を更新していない
- ユーザーの環境では反響は正常に動いている

## DLSS / NR（OptiScaler の NR フォーク、wilsjo2 版）

- NR を使うには `Dx11Upscaler=dlss_12` が必要。ただの `dlss` では NR は動かない
- NR のプリセット 2（Optimized）を使うには、NR ランタイム 310.9.16 が必要
- ini で preset 2 に設定済み。描画負荷は 1.95ms まで下がった

## 進め方の決定事項

- ユーザーへの返答は**日本語**でする（英語に戻らないよう注意）
- MOD 作者や Discord 向けの文は、英語の文に日本語の要約を付けて出す
- テキストを書き換えるときは、改行コード（CRLF）を保つためにバイト単位で扱う
- エンジンを修正したら、フォークに push して Actions でビルドし、ユーザーがゲーム内で確認する
- 上流（v0.4 の作者）に報告するときは、原因と修正の最小限の差分を伝える
