using System;
using System.Linq;

namespace SimplySupportedBeamReinforcement_DirectCrackWidth
{
    // Struct to hold results from SLS stress calculation
    public struct SLSStressCalcResult
    {
        public double Sigma_s_MPa { get; }
        public double X_NA_Cracked_mm { get; } // Depth of neutral axis from compression face
        public bool IsCracked { get; }

        public SLSStressCalcResult(double sigma_s, double x_na_cracked, bool isCracked)
        {
            Sigma_s_MPa = sigma_s;
            X_NA_Cracked_mm = x_na_cracked;
            IsCracked = isCracked;
        }
    }

    class Program
    {
        // --- Design Parameters (User Modifiable) ---
        static double b = 300.0;   // Section width (mm)
        static double h = 550.0;   // Total depth (mm)
                                   // IMPORTANT: 'cover' is now defined as CLEAR CONCRETE COVER to the main longitudinal tension bars.
        static double cover_clear_main_bar = 35.0;

        // Loads
        static double MEd_ULS = 100.0;  // ULS Design Moment [kNm]
        static double VEd_ULS = 80.0;   // ULS Design Shear [kN]
        static double MSlk_SLS_QuasiPerm = 70.0; // SLS Quasi-Permanent Moment [kNm] (for crack width and long-term deflections)
        static double L_span = 6000.0; // Beam span [mm]

        // Material Properties
        static double f_ck = 30.0;  // MPa
        static double f_yk = 500.0; // MPa

        // Crack Control Limit
        static double w_k_limit_mm = 0.3; // Allowable characteristic crack width (mm)

        // --- Eurocode Parameters (Finnish National Annex - SFS-EN 1992-1-1 NA) ---
        static double gamma_c = 1.5;
        static double gamma_s = 1.15;
        static double alpha_cc = 0.85;
        static double f_cd = alpha_cc * f_ck / gamma_c;
        static double f_sd = f_yk / gamma_s;
        static double E_s = 200000.0; // MPa
        static double cot_theta_shear = 1.0;

        // Reinforcement Options
        static int[] topBarDiameters = { 12, 16 };
        static int[] topBarCounts = { 2 };
        static int[] botBarDiameters = { 16, 20, 25 };
        static int[] botBarCounts = { 2, 3, 4 };
        static (int phi_stirrup, int legs, double spacing)[] shearConfigs =
        {
(8,  2, 200), (8,  2, 150),
(10, 2, 200), (10, 2, 150)
};

        static void Main(string[] args)
        {
            Console.WriteLine($"Beam Design (Finnish NA - Direct Crack Width Calculation w_k <= {w_k_limit_mm}mm):");
            Console.WriteLine($"b={b}mm, h={h}mm, clear_cover_main_bar={cover_clear_main_bar}mm, L_span={L_span}mm");
            Console.WriteLine($"MEd_ULS={MEd_ULS}kNm, VEd_ULS={VEd_ULS}kN, MSlk_SLS_QuasiPerm={MSlk_SLS_QuasiPerm}kNm");
            // ... (rest of the initial parameter printout)

            double bestOverallUtil = double.MaxValue;
            string bestConfigDetails = "(No suitable configuration found)";

            string header_line_top = "+-------+-------+--------+------+------+-----+-----------+--------+--------+";
            string header_titles = "|Top Rb.|Bot Rb.|Shear   |Mom.U.|Shr.U.|Asmin|CrackWk mm |Defl L/d|Max Util|";
            string header_details_units = "| (nxØ) | (nxØ) |(LØ@S)  |(ULS) |(ULS) |(SLS)| (w_k)     | (A/Max)|        |";

            Console.WriteLine(header_line_top);
            Console.WriteLine(header_titles);
            Console.WriteLine(header_details_units);
            Console.WriteLine(header_line_top);

            foreach (int phiTop in topBarDiameters)
            {
                foreach (int nTop in topBarCounts)
                {
                    foreach (int phiBot in botBarDiameters)
                    {
                        foreach (int nBot in botBarCounts)
                        {
                            foreach (var sh_cfg in shearConfigs)
                            {
                                double AsTop_mm2 = nTop * (Math.PI * phiTop * phiTop / 4.0);
                                double AsBot_mm2 = nBot * (Math.PI * phiBot * phiBot / 4.0);

                                // Effective depth to centroid of bottom tension steel
                                double dBot_eff = h - (cover_clear_main_bar + phiBot / 2.0);
                                // Effective depth to centroid of top compression steel (if any)
                                double dTop_eff = cover_clear_main_bar + phiTop / 2.0;


                                // --- ULS Checks ---
                                double MRd_Nmm = BendingCapacitySinglyReinforced(b, dBot_eff, f_cd, f_sd, AsBot_mm2);
                                double MRd_kNm = MRd_Nmm * 1e-6;
                                double utilMoment_ULS = MEd_ULS / Math.Max(MRd_kNm, 1e-6);
                                bool okMoment_ULS = utilMoment_ULS <= 1.005;

                                double VRdc_kN = ConcreteShearCapacity_EC2_Cl6_2_2(b, dBot_eff, f_ck, gamma_c, AsBot_mm2);
                                double Asw_per_stirrup_mm2 = sh_cfg.legs * (Math.PI * sh_cfg.phi_stirrup * sh_cfg.phi_stirrup / 4.0);
                                double z_shear_approx = 0.9 * dBot_eff;
                                double VRds_kN = (Asw_per_stirrup_mm2 / sh_cfg.spacing) * z_shear_approx * (f_yk / gamma_s) * cot_theta_shear * 1e-3;
                                double nu1 = 0.6 * (1 - f_ck / 250.0);
                                double alpha_cw_shear = 1.0;
                                double V_Rd_max_kN = (alpha_cw_shear * b * z_shear_approx * nu1 * f_cd) / (cot_theta_shear + (1.0 / cot_theta_shear)) * 1e-3;
                                double VRd_total_kN = Math.Min(VRdc_kN + VRds_kN, V_Rd_max_kN);
                                double utilShear_ULS = VEd_ULS / Math.Max(VRd_total_kN, 1e-6);
                                bool okShear_ULS = utilShear_ULS <= 1.005;

                                // --- SLS Checks ---
                                double f_ctm = Calculate_f_ctm(f_ck);
                                double E_cm = Calculate_E_cm(f_ck);

                                // 1. Minimum Reinforcement Area
                                double As_min1 = 0.26 * (f_ctm / f_yk) * b * dBot_eff;
                                double As_min2 = 0.0013 * b * dBot_eff;
                                double As_min_req_mm2 = Math.Max(As_min1, As_min2);
                                bool okAsMin_SLS = AsBot_mm2 >= As_min_req_mm2;

                                // 2. Crack Width Calculation (Direct Method EC2 7.3.4)
                                SLSStressCalcResult slsStressResult = CalculateSteelStress_SLS(MSlk_SLS_QuasiPerm * 1e6, b, h, dBot_eff, AsBot_mm2, dTop_eff, AsTop_mm2, f_ck, E_s, f_ctm, E_cm);
                                double w_k_calculated_mm = 0;
                                bool okCrackWidth_SLS = true; // Assume OK if not cracked or no tension steel

                                if (slsStressResult.IsCracked && AsBot_mm2 > 1e-6)
                                {
                                    double rho_p_eff = Calculate_rho_p_eff_EC2(b, h, AsBot_mm2, slsStressResult.X_NA_Cracked_mm, cover_clear_main_bar, phiBot);
                                    if (rho_p_eff > 1e-9) // Ensure rho_p_eff is meaningful
                                    {
                                        double s_r_max_mm = Calculate_sr_max_EC2(cover_clear_main_bar, phiBot, rho_p_eff);
                                        double eps_sm_minus_eps_cm = Calculate_epsilon_sm_minus_epsilon_cm_EC2(slsStressResult.Sigma_s_MPa, f_ctm, rho_p_eff, E_s, E_cm);
                                        w_k_calculated_mm = s_r_max_mm * eps_sm_minus_eps_cm;
                                        okCrackWidth_SLS = w_k_calculated_mm <= w_k_limit_mm;
                                    }
                                    else
                                    {
                                        okCrackWidth_SLS = false; // rho_p_eff too small, likely problematic
                                        w_k_calculated_mm = 999; // Indicate error
                                    }
                                }
                                else if (!slsStressResult.IsCracked)
                                {
                                    w_k_calculated_mm = 0; // No cracks if section uncracked by MSlk_QuasiPerm
                                    okCrackWidth_SLS = true;
                                }


                                // 3. Deflection Control
                                double L_d_actual = L_span / dBot_eff;
                                double As_req_ULS_MEd_mm2 = CalculateRequiredSteelForMoment_ULS(MEd_ULS * 1e6, b, dBot_eff, f_cd, f_sd);
                                double L_d_allowable = CalculateAllowable_L_d_ratio_EC2(L_span, b, dBot_eff, AsBot_mm2, AsTop_mm2, f_ck, f_yk, As_req_ULS_MEd_mm2);
                                bool okDeflection_SLS = L_d_actual <= L_d_allowable;

                                // Overall Status
                                bool all_ULS_OK = okMoment_ULS && okShear_ULS;
                                bool all_SLS_OK = okAsMin_SLS && okCrackWidth_SLS && okDeflection_SLS;
                                // ... (Best config tracking logic - same as before) ...
                                double currentMaxUtilNum = (all_ULS_OK && all_SLS_OK) ? Math.Max(utilMoment_ULS, utilShear_ULS) : double.MaxValue;
                                if (currentMaxUtilNum < bestOverallUtil)
                                {
                                    bestOverallUtil = currentMaxUtilNum;
                                    bestConfigDetails = $"Top: {nTop}x{phiTop}, Bot: {nBot}x{phiBot}, Stir: {sh_cfg.legs}Ø{sh_cfg.phi_stirrup}@{sh_cfg.spacing} | ULS M:{utilMoment_ULS:F2}, V:{utilShear_ULS:F2} | SLS w_k:{w_k_calculated_mm:F2}mm, All OK";
                                }


                                // --- Output Strings ---
                                string crackWkStr = okCrackWidth_SLS ? $"{w_k_calculated_mm:F2}" : $"FAIL {w_k_calculated_mm:F2}";
                                if (!slsStressResult.IsCracked && AsBot_mm2 > 1e-6) crackWkStr = "0.00 (Uncr)"; // Special case for uncracked

                                string topRbStr = $"{nTop}x{phiTop}";
                                string botRbStr = $"{nBot}x{phiBot}";
                                string shearStr = $"{sh_cfg.legs}Ø{sh_cfg.phi_stirrup}@{sh_cfg.spacing}";
                                string momUStr = $"{utilMoment_ULS:F2}{(okMoment_ULS ? "" : "*")}";
                                string shrUStr = $"{utilShear_ULS:F2}{(okShear_ULS ? "" : "*")}";
                                string asMinStr = okAsMin_SLS ? "OK" : "FAIL";
                                string deflStr = okDeflection_SLS ? "OK" : $"FAIL {L_d_actual:F0}/{L_d_allowable:F0}";
                                string maxUtilFinalStr = (all_ULS_OK && all_SLS_OK) ? $"{currentMaxUtilNum:F2}" : (all_ULS_OK ? "SLS_FAIL" : "ULS_FAIL");


                                Console.WriteLine($"|{topRbStr,-7}|{botRbStr,-7}|{shearStr,-8}|{momUStr,-6}|{shrUStr,-6}|{asMinStr,-5}|{crackWkStr,-11}|{deflStr,-8}|{maxUtilFinalStr,-8}|");
                            }
                        }
                    }
                }
            }
            // ... (rest of Main method - closing table, best config printout)
            Console.WriteLine(header_line_top);
            Console.WriteLine("\nBest Acceptable Solution:");
            Console.WriteLine(bestConfigDetails);
            Console.WriteLine("\nNote: (Uncr) means section uncracked under quasi-permanent SLS moment for w_k check.");
            Console.WriteLine("\nPress any key to exit.");
            Console.ReadKey();
        }

        // --- Crack Width Calculation Helper Functions (EC2 7.3.4) ---
        static double Calculate_rho_p_eff_EC2(double b_beam, double h_beam, double As_tension_steel, double x_NA_sls_cracked_mm, double c_clear_tension_bar_mm, double phi_tension_bar_mm)
        {
            // Effective height of concrete in tension, h_c,eff (EC2 7.3.2 (3))
            // (h-d) is distance from tension fibre to centroid of tension steel = c_clear + phi/2
            double dist_tension_face_to_As_centroid = c_clear_tension_bar_mm + phi_tension_bar_mm / 2.0;

            double h_c_eff_comp1 = 2.5 * dist_tension_face_to_As_centroid;
            double h_c_eff_comp2 = (h_beam - x_NA_sls_cracked_mm) / 3.0;
            double h_c_eff_comp3 = h_beam / 2.0;

            double h_c_eff = Math.Min(h_c_eff_comp1, Math.Min(h_c_eff_comp2, h_c_eff_comp3));
            if (h_c_eff < 1e-6) return 0; // Avoid division by zero if h_c_eff is somehow zero

            double A_c_eff = b_beam * h_c_eff;
            if (A_c_eff < 1e-6) return 0;

            return As_tension_steel / A_c_eff;
        }

        static double Calculate_sr_max_EC2(double c_clear_mm, double phi_tension_bar_mm, double rho_p_eff)
        {
            // Max crack spacing s_r,max (EC2 Eq. 7.11, with k3, k4 from 7.3.3(3) for bar spacing rules)
            double k1 = 0.8; // High bond bars
            double k2 = 0.5; // Bending
            double k3 = 3.4;
            double k4 = 0.425;

            // Ensure rho_p_eff is not excessively small to prevent extreme s_r,max values
            double effective_rho_p_eff = Math.Max(rho_p_eff, 0.001); // Minimum sensible rho_p_eff for formula stability

            return k3 * c_clear_mm + (k1 * k2 * k4 * phi_tension_bar_mm) / effective_rho_p_eff;
        }

        static double Calculate_epsilon_sm_minus_epsilon_cm_EC2(double sigma_s_MPa, double f_ct_eff_MPa, double rho_p_eff, double E_s_MPa, double E_cm_MPa)
        {
            // Difference in mean strain (EC2 Eq. 7.9)
            double k_t = 0.4; // Factor dependent on duration of load (0.4 for long-term)
            double alpha_e = E_s_MPa / E_cm_MPa;

            double term1_numerator = sigma_s_MPa - k_t * (f_ct_eff_MPa / rho_p_eff) * (1 + alpha_e * rho_p_eff);
            double term1 = term1_numerator / E_s_MPa;

            double term2 = 0.6 * sigma_s_MPa / E_s_MPa;

            return Math.Max(term1, term2);
        }


        // --- Existing Helper Functions (Modified SLS Stress Calc) ---
        static SLSStressCalcResult CalculateSteelStress_SLS(double M_sls_Nmm, double B_width, double H_total,
        double d_bot_eff, double As_bot_mm2,
        double d_top_eff, double As_top_mm2,
        double fck, double Es_steel, double f_ctm_concrete, double E_cm_concrete)
        {
            if (As_bot_mm2 <= 1e-6 && M_sls_Nmm > 0)
            { // No tension steel but moment exists
                return new SLSStressCalcResult(Es_steel * 0.01, H_total, true); // High stress, assume cracked, NA at top
            }
            if (As_bot_mm2 <= 1e-6 && M_sls_Nmm <= 0)
            {
                return new SLSStressCalcResult(0, H_total / 2.0, false);
            }


            double alpha_e = Es_steel / E_cm_concrete;

            // Cracking Moment M_cr
            double I_gross = B_width * Math.Pow(H_total, 3) / 12.0;
            double y_t_gross = H_total / 2.0; // Distance from centroid to extreme tension fiber
            double M_cr_Nmm = f_ctm_concrete * I_gross / y_t_gross;

            double sigma_s_MPa;
            double x_NA_mm; // Neutral axis depth from compression face
            bool isCracked;

            if (M_sls_Nmm <= M_cr_Nmm) // Section Uncracked (State I)
            {
                isCracked = false;
                // Transformed section properties
                double A_transformed_top = (alpha_e - 1) * As_top_mm2;
                double A_transformed_bot = (alpha_e - 1) * As_bot_mm2;

                double A_total_transformed = B_width * H_total + A_transformed_top + A_transformed_bot;
                // Centroid of transformed section from top compression face
                double y_bar_transformed = ((B_width * H_total * (H_total / 2.0)) +
                (A_transformed_top * d_top_eff) +
                (A_transformed_bot * d_bot_eff)) / A_total_transformed;
                x_NA_mm = y_bar_transformed; // For uncracked, NA is at centroid of transformed section

                // Moment of inertia of transformed uncracked section about its centroid
                double I_transformed = (B_width * Math.Pow(H_total, 3) / 12.0 + B_width * H_total * Math.Pow(H_total / 2.0 - y_bar_transformed, 2)) +
                (A_transformed_top * Math.Pow(d_top_eff - y_bar_transformed, 2)) +
                (A_transformed_bot * Math.Pow(d_bot_eff - y_bar_transformed, 2));

                sigma_s_MPa = alpha_e * (M_sls_Nmm * (d_bot_eff - y_bar_transformed)) / Math.Max(I_transformed, 1e-6);
            }
            else // Section Cracked (State II)
            {
                isCracked = true;
                // Solve for neutral axis depth x_NA_mm (quadratic equation)
                // B*x^2/2 + alpha_e*As_top*(x - d_top) - alpha_e*As_bot*(d_bot - x) = 0
                // 0.5*B*x^2 + alpha_e*(As_top + As_bot)*x - alpha_e*(As_top*d_top + As_bot*d_bot) = 0
                double a_quad = 0.5 * B_width;
                double b_quad = alpha_e * (As_top_mm2 + As_bot_mm2);
                double c_quad = -alpha_e * (As_top_mm2 * d_top_eff + As_bot_mm2 * d_bot_eff);

                double discriminant = b_quad * b_quad - 4 * a_quad * c_quad;
                if (discriminant < 0)
                { // Should not happen with typical inputs
                    return new SLSStressCalcResult(Es_steel * 0.01, H_total / 2.0, true); // Error, return high stress
                }

                x_NA_mm = (-b_quad + Math.Sqrt(discriminant)) / (2 * a_quad);
                // Basic validation for x_NA_mm
                x_NA_mm = Math.Max(Math.Min(d_top_eff, 1e-3), Math.Min(x_NA_mm, d_bot_eff * 0.99)); // Ensure x is within reasonable bounds and positive

                // Moment of inertia of cracked section about the neutral axis
                double I_cracked = (B_width * Math.Pow(x_NA_mm, 3) / 3.0) +
                (alpha_e * As_top_mm2 * Math.Pow(x_NA_mm - d_top_eff, 2)) + // As_top contributes if x_NA > d_top_eff
                (alpha_e * As_bot_mm2 * Math.Pow(d_bot_eff - x_NA_mm, 2));

                if (I_cracked <= 1e-6) return new SLSStressCalcResult(Es_steel * 0.01, x_NA_mm, true); // Error, return high stress

                sigma_s_MPa = alpha_e * (M_sls_Nmm * (d_bot_eff - x_NA_mm)) / I_cracked;
            }

            return new SLSStressCalcResult(Math.Min(sigma_s_MPa, f_yk), x_NA_mm, isCracked);
        }

        // --- Other existing helper functions (BendingCapacity, RequiredSteel, ShearCapacity, f_ctm, E_cm, L/d ratio) ---
        // These are assumed to be mostly correct from previous versions but may need slight adjustments if 'cover' definition impacted them.
        // For brevity, they are not all repeated here but should be the same as the last "FAIL" version,
        // ensuring 'cover_clear_main_bar' is used consistently where clear cover is needed.

        static double BendingCapacitySinglyReinforced(double B, double d_eff, double fcd, double fsd, double As_prov)
        {
            if (As_prov <= 1e-9) return 0;
            double lambda = (f_ck <= 50) ? 0.8 : 0.8 - (f_ck - 50.0) / 400.0;
            double s_stress_block = (As_prov * fsd) / (fcd * B);
            double z_lever_arm = d_eff - (s_stress_block / 2.0);
            if (z_lever_arm < 0.05 * d_eff) z_lever_arm = 0.05 * d_eff;
            return Math.Max(As_prov * fsd * z_lever_arm, 0);
        }
        static double CalculateRequiredSteelForMoment_ULS(double M_uls_Nmm, double B, double d_eff, double fcd, double fsd)
        {
            if (M_uls_Nmm <= 0) return 0;
            double K_limit_ductile = (f_ck <= 50) ? 0.167 : (0.167 * (1 - (f_ck - 50) / 200.0));
            double K = M_uls_Nmm / (B * d_eff * d_eff * fcd);
            if (K > K_limit_ductile) M_uls_Nmm = K_limit_ductile * B * d_eff * d_eff * fcd;
            double term_under_sqrt_numerator = 2.0 * M_uls_Nmm;
            double term_under_sqrt_denominator = fcd * B;
            if (term_under_sqrt_denominator == 0) return double.MaxValue;
            double term_under_sqrt = d_eff * d_eff - (term_under_sqrt_numerator / term_under_sqrt_denominator);
            if (term_under_sqrt < 0) return double.MaxValue;
            double s_req = d_eff - Math.Sqrt(term_under_sqrt);
            if (double.IsNaN(s_req) || s_req < 0) return double.MaxValue;
            double As_req = (fcd * B * s_req) / fsd;
            return Math.Max(0, As_req);
        }
        static double ConcreteShearCapacity_EC2_Cl6_2_2(double B, double d_eff, double fck, double gammaC, double As_long_tens_mm2)
        {
            double C_Rd_c = 0.18 / gammaC;
            double k_defl = Math.Min(1.0 + Math.Sqrt(200.0 / d_eff), 2.0);
            double rho_l_tens = Math.Min(As_long_tens_mm2 / (B * d_eff), 0.02);
            double v_rd_c_val1 = C_Rd_c * k_defl * Math.Pow(100.0 * rho_l_tens * fck, 1.0 / 3.0);
            double v_min_coeff = 0.035;
            double v_min_val = v_min_coeff * Math.Pow(k_defl, 1.5) * Math.Pow(fck, 0.5);
            double v_rd_c_final = Math.Max(v_rd_c_val1, v_min_val);
            return (v_rd_c_final * B * d_eff) * 1e-3;
        }
        static double Calculate_f_ctm(double fck)
        {
            if (fck <= 50) return 0.30 * Math.Pow(fck, 2.0 / 3.0);
            else return 2.12 * Math.Log(1 + (fck + 8.0) / 10.0);
        }
        static double Calculate_E_cm(double fck)
        {
            double fcm = fck + 8.0;
            return 22000.0 * Math.Pow(fcm / 10.0, 0.3);
        }
        static double CalculateAllowable_L_d_ratio_EC2(double span_L_mm, double B_width, double d_bot_eff, double As_bot_provided_mm2, double As_top_provided_mm2, double fck, double fyk_steel, double As_bot_required_ULS_mm2)
        {
            if (d_bot_eff <= 1e-6) return 0.1;
            double K = 1.0;
            double rho_bot = As_bot_provided_mm2 / (B_width * d_bot_eff);
            double rho_top = As_top_provided_mm2 / (B_width * d_bot_eff);
            double rho_0_ref = Math.Sqrt(fck) * 1e-3;
            double term_comp_steel = 0;
            if (rho_top > 1e-9 && rho_0_ref > 1e-9)
            {
                term_comp_steel = (1.0 / 12.0) * Math.Sqrt(fck) * Math.Sqrt(rho_top / rho_0_ref);
            }
            double L_d_base = K * (11.0 + (1.5 * Math.Sqrt(fck) * rho_0_ref) / Math.Max(rho_bot, 1e-9 * rho_0_ref) + term_comp_steel);
            double factor_As_ratio = 1.0;
            if (As_bot_required_ULS_mm2 > 1e-6)
            {
                factor_As_ratio = As_bot_provided_mm2 / As_bot_required_ULS_mm2;
                if (factor_As_ratio < 0.1) factor_As_ratio = 0.1;
                factor_As_ratio = Math.Min(factor_As_ratio, 1.5);
            }
            return L_d_base * factor_As_ratio;
        }
    }
}