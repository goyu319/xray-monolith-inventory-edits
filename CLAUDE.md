# CLAUDE.md — GAMMA カスタムエンジン & MOD 環境

S.T.A.L.K.E.R. Anomaly + GAMMA 環境の作業メモ（エンジン開発部分）。このファイルには、これまでの作業の経緯と決定事項、次に作業するときの注意点をまとめる。

## 環境

- S.T.A.L.K.E.R. Anomaly + GAMMA（MO2 プロファイルは Squared Away）
- **使う exe**：goyu-v051 では **AnomalyDX12.exe**（旧ブランチでは AnomalyDX11AVX.exe）
  - 以前 AnomalyDX12.exe が Squared Away の assert で落ちたのは、本家の exe に Squared Away 用の改造が入っていなかったため
- PC 環境や自作 MOD のメモは、公開しないためにリポジトリ外の個人用メモ（CLAUDE.local.md）に分けてある。必要ならユーザーに貼ってもらう

## 現行ブランチ：`goyu-v051`（2026-11-06〜）

- **土台**：本家 smoke-0n-the-water の **v0.5.1-dev**（brodrigz DLSS 0.71、DX12 フレーム生成、性能改善、Immersive Bodycam 0.3.1 入り）
- **使う exe**：**AnomalyDX12.exe**（本家の FG は DX12 ブリッジ版の exe でしか動かない）
- **上に重ねたもの**
  1. `Revert` 2件：フレームレート上限 2000（`c71f74961`）、起動画面（`0ad6f845a`）。ユーザーの希望で外した
     - Discord（`5de62e4c7`）は外すと「Quark Engine」名義＋招待ボタンになるので、本家の版（ストックの Anomaly 表示）のまま
  2. Fomok の Squared Away 対応（`1cbced1b7`）と、レベルパック読み込みの版数対応（`fc3e34ff6`）
  3. 武器が真っ黒になる修正（`470591042`）。本家 v0.5.1 にも同じバグが残っていた
  4. フリールックの修正 4 件（`755f2f809` `eaba5d776` `0b2c269a6` `75a7c02ca`）
  5. 本家（demonized）2026.11.9 の #759 だけ cherry-pick（`f5c009e67`）：ロード中に Alt+Tab すると雨などの環境音が消える問題の修正。2026.11.9 全体のマージは描画まわりで 75 ファイル衝突するので見送り
  6. 本家 2026.11.9 から不具合修正を追加で cherry-pick（2026-11-12、Actions 43153700174 でビルド成功）
     - そのまま：#745 #746 #764 #771 #763 #760 #765 #823 #820 #824 #815 #813 #811 #808
     - 手で移植：#818（`Feel_Vision.cpp`、`5783d21f3`）
     - #784 は取り込み済みだった。#827 は武器アタッチメント系の CHUDState メンバーが要るのでビルドが通らず revert（`43c11c96e`）
     - 見送り：武器アタッチメント系（#714 #719 #741 #724 #740 #817）、#739、#762、#742、MOD 用の新機能（#748 #757 #782 #752 #758）、#816
  7. 服を剥いだ NPC でのクラッシュ修正（`be9e5121d`、Actions 43168127093）：`CObject::cNameVisual_set` で古いモデルを消す前に骨格ウォールマークを外す（`CKinematics::RenderWallmark` のダングリングポインタ）
  8. `hud_fov` の上限を 1.0 → 1.5（`7fd0027c6`、Actions 43189670747）。GOYU All-in-One のマガジン装填アニメ用 FOV スライダー（0.10〜1.50）のため
  9. 本家 **v0.6-dev**（`d22882ed5`）をマージ（2026-11-13）：DLSS/FSR 0.72、PiP スコープ中の FG（`r_frame_generation_pip`）とレンズのモーションベクター、VRAM 警告で FG が落ちない、AI の経路修正、物理クラッシュ対策、テクスチャ常駐の改善、#825・#827（作り直し版）ほか。衝突は起動画面の画像だけで、GOYU 側を残した。武器が真っ黒になる修正は本家 v0.6 にもまだ無いので引き続き必要
     - mod 側は DLSS 0.72 と Compat Patch 0.72 が必要。v0.6 のインストーラーは MO2 の modlist や起動設定を書き換えるので使わず、必要な MOD だけ手で入れる
  10. ショットガンがケースの弾でリロードできない問題の修正（2026-11-14）：`CWeapon::GetAmmoCount_forType` がバッグとベルトしか数えず、ケース（`CInventoryContainer`）の中の弾を数えていなかった。弾を取り出す `CInventory::GetAny` はケースの中も探すので、数と実際が食い違っていた。持ち歩いているケースの中の弾も数えるようにした
- **入れていないもの（不要になった）**
  - 屋内銃声の修正：本家に取り込み済み
  - デバイス所持で FG が止まる修正：本家の FG（`FrameGeneration.cpp` の `Eligible()`）にはその停止条件がない
  - Fomok の Personal FG 一式（DX11 から DX12 提示のブリッジ、スコープ/PiP の FG 修正）：本家の DLSS 0.71 の仕組みに置き換え
- **ビルド**：`.github/workflows/goyu-build.yml`（push で自動。成果物 `GOYU-Engine-v051-SquaredAway-DX12`）
- **入れ方**：本家 v0.5.1-dev のパックを先に入れ（Streamline や FSR の DLL、DLSS 0.71 の MOD が必要）、そのあと bin と db をこの成果物で上書き
- **ユーザー環境での設定（2026-11-06 確認）**：RTX 5070 / Ryzen 7 9800X3D / 2560x1440。FG は **DLSS 3x**、画質は **Quality + プリセット K** で運用
  - FG の動作はログで確認済み（`Interpolation enabled (3x)`、PostPresent の `presented=3`）
  - `r__framelimit` は FG 後の**表示フレーム数**にかかる（Reflex の上限 = 倍率 / limit）。上限 149 のままだと 2x でも 149 で頭打ちになる。今は 499（最大 500。2000 にする本家コミットは外している）
  - 3x/4x では元の描画が大きく落ちる（3x で約 50、4x で約 30）。VRAM 予算を下げても変わらなかったので、5070 での FG 処理の負荷と判断
  - ゲーム内の FPS 表示は描画フレームのみ。表示フレームは NVIDIA オーバーレイで見る
  - `r_fg_diagnostics 1`（起動時に確定）で PostPresent の `presented=` がログに出る
  - 既知・対応しない：PiP スコープを覗いて動かすとざらつく（本家ではスコープ映像が時間方向の再構成を通らない。Fomok 版の PiP 修正は移植していない。ユーザー了承済み）
- **MO2 の構成（2026-11-06）**：本家パックの 3（Compat Patch remerged 0.71）、4（core v0.5.1）、6（Scope Effects Guard v0.5.1）、Add-on（RC7、Glossy、Better Beef、Anti-Lag、Viper 2）を有効。GSquared.Away.FG.UI.Scope.Patch、GSA031、GDLSS FSR3 Compatibility Patch、旧 G6、旧 3/4/5 は無効。変更前は `modlist.txt.before_v051`
- **旧ブランチ**：`sa-fg-v04-bc031` / `freelook-keep-view`（Fomok ベース、AnomalyDX11AVX.exe）。下の記録はその時代のもの

## エンジン（旧：sa-fg-v04-bc031）

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
8. **自前修正：デバイス所持で FG が止まるバグ**（`d456ce7`）。下の「デバイス所持で FG が止まる件」を参照
   - Actions の実行 41274692429 でビルド成功、ゲーム内で直ったことを確認済み（2026-11-01）。`sa-fg-v04-bc031` にマージ済み

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

### デバイス所持で FG が止まる件（2026-11-01 に解決）

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
